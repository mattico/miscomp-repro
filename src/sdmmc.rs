use log::{info};
use core::cell::RefCell;
use stm32h7xx_hal::hal::digital::v2::{InputPin, OutputPin, ToggleableOutputPin};
use stm32h7xx_hal::{gpio::*, pac, prelude::*, rcc, rcc::rec, sdmmc};

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
        info!("MicroSD Card connected: {:?}", card_info);

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
