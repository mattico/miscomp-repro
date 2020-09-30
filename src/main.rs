//! SDMMC card example

#![no_main]
#![no_std]

mod logger;
mod sdmmc;

use core::cell::RefCell;
use cortex_m::asm;
use cortex_m::interrupt::Mutex;
use cortex_m_rt::entry;
use stm32h7xx_hal::{interrupt, pac, prelude::*};

use log::{error, info};

static SDMMC: Mutex<RefCell<Option<sdmmc::Sdmmc>>> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    logger::init();
    let _cp = cortex_m::Peripherals::take().unwrap();
    let mut dp = pac::Peripherals::take().unwrap();

    // Constrain and Freeze power
    info!("Setup PWR...                  ");
    let pwr = dp.PWR.constrain();
    let pwrcfg = pwr.freeze();

    // Constrain and Freeze clock
    info!("Setup RCC...                  ");
    let rcc = dp.RCC.constrain();

    let ccdr = rcc
        .sys_ck(400.mhz())
        .pll1_q_ck(100.mhz())
        .freeze(pwrcfg, &dp.SYSCFG);

    let gpioa = dp.GPIOA.split(ccdr.peripheral.GPIOA);
    let gpiob = dp.GPIOB.split(ccdr.peripheral.GPIOB);
    gpiob.pb3.into_alternate_af0(); // TRACESWO
    let gpioc = dp.GPIOC.split(ccdr.peripheral.GPIOC);
    let gpiod = dp.GPIOD.split(ccdr.peripheral.GPIOD);

    let mut sdmmc = sdmmc::Sdmmc::new(
        gpioc.pc12,
        gpiod.pd2,
        gpioc.pc8,
        gpioc.pc9,
        gpioc.pc10,
        gpioc.pc11,
        gpioc.pc6,
        gpioa.pa11,
        dp.SDMMC1,
        ccdr.peripheral.SDMMC1,
        &mut dp.SYSCFG,
        &mut dp.EXTI,
        &ccdr.clocks,
    );
    if sdmmc.is_detected() {
        if let Err(e) = sdmmc.init_card() {
            error!("Unable to connect to MicroSD card: {:?}", e);
        }
    }

    cortex_m::interrupt::free(|cs| {
        SDMMC.borrow(cs).replace(Some(sdmmc));
    });

    unsafe {
        pac::NVIC::unmask(interrupt::EXTI9_5);
    }

    loop {
        asm::wfi();
    }
}

#[interrupt]
fn EXTI9_5() {
    cortex_m::interrupt::free(|cs| {
        let mut sdmmc = SDMMC.borrow(cs).borrow_mut();
        let sdmmc = sdmmc.as_mut().unwrap();
        if sdmmc.handle_detect() {
            info!("MicroSD card inserted");
            if let Err(e) = sdmmc.init_card() {
                error!("Error connecting to MicroSD card: {:?}", e);
            }
        } else {
            info!("MicroSD card removed");
        }
    });
}
