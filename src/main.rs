//! SDMMC card example

#![no_main]
#![no_std]

use core::cell::RefCell;
use cortex_m::asm;
use cortex_m::interrupt::Mutex;
use cortex_m_rt::entry;
use panic_rtt_target as _;
use rtt_target::{rprintln, rtt_init_print};
use stm32h7xx_hal::{interrupt, pac, prelude::*};

static SDMMC: Mutex<RefCell<Option<Sdmmc>>> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    rtt_init_print!();
    let _cp = cortex_m::Peripherals::take().unwrap();
    let mut dp = pac::Peripherals::take().unwrap();

    rprintln!("");
    rprintln!("Startup");
    rprintln!("");

    // Constrain and Freeze power
    rprintln!("Setup PWR...                  ");
    let pwr = dp.PWR.constrain();
    let pwrcfg = pwr.freeze();

    // Constrain and Freeze clock
    rprintln!("Setup RCC...                  ");
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

    let mut sdmmc = Sdmmc::new(
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
            rprintln!("Unable to connect to MicroSD card: {:?}", e);
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
            rprintln!("MicroSD card inserted");
            if let Err(e) = sdmmc.init_card() {
                rprintln!("Error connecting to MicroSD card: {:?}", e);
            }
        } else {
            rprintln!("MicroSD card removed");
        }
    });
}

use stm32h7xx_hal::hal::digital::v2::{InputPin, OutputPin};
use stm32h7xx_hal::{gpio::*, prelude::*, rcc, rcc::rec, sdmmc};

pub struct Sdmmc {
    sdmmc: sdmmc::Sdmmc<pac::SDMMC1>,
    detect: gpioc::PC6<Input<PullDown>>,
    led: gpioa::PA11<Output<OpenDrain>>,
}

impl Sdmmc {
    pub fn new(
        clk: gpioc::PC12<Analog>,
        cmd: gpiod::PD2<Analog>,
        d0: gpioc::PC8<Analog>,
        d1: gpioc::PC9<Analog>,
        d2: gpioc::PC10<Analog>,
        d3: gpioc::PC11<Analog>,
        detect: gpioc::PC6<Analog>,
        led: gpioa::PA11<Analog>,
        sdmmc: pac::SDMMC1,
        sdprec: rec::Sdmmc1,
        syscfg: &mut pac::SYSCFG,
        exti: &mut pac::EXTI,
        clocks: &rcc::CoreClocks,
    ) -> Self {
        let clk = clk.into_alternate_af12().set_speed(Speed::VeryHigh);
        let cmd = cmd.into_alternate_af12().set_speed(Speed::VeryHigh);
        let d0 = d0.into_alternate_af12().set_speed(Speed::VeryHigh);
        let d1 = d1.into_alternate_af12().set_speed(Speed::VeryHigh);
        let d2 = d2.into_alternate_af12().set_speed(Speed::VeryHigh);
        let d3 = d3.into_alternate_af12().set_speed(Speed::VeryHigh);

        let mut detect = detect.into_pull_down_input();
        detect.make_interrupt_source(syscfg);
        detect.trigger_on_edge(exti, Edge::RISING_FALLING);
        detect.enable_interrupt(exti);

        let mut led = led.into_open_drain_output();
        led.set_high().unwrap();

        let sdmmc = sdmmc.sdmmc((clk, cmd, d0, d1, d2, d3), sdprec, clocks);

        Sdmmc { sdmmc, detect, led }
    }

    /// Attempts to connect to an SDMMC card
    pub fn init_card(&mut self) -> Result<(), sdmmc::Error> {
        self.led.set_high().unwrap();
        if self.detect.is_low().unwrap() {
            return Err(sdmmc::Error::NoCard);
        }
        self.sdmmc.init_card(50.mhz())?;
        let card_info = self.sdmmc.card()?;
        self.led.set_low().unwrap();
        rprintln!("MicroSD Card connected: {:?}", card_info);

        Ok(())
    }

    pub fn is_detected(&self) -> bool {
        self.detect.is_high().unwrap()
    }

    /// Clears the IRQ for the detect pin and returns `true` if a MicroSD card is inserted.
    pub fn handle_detect(&mut self) -> bool {
        self.detect.clear_interrupt_pending_bit();
        self.detect.is_high().unwrap()
    }
}
