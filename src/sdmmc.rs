use log::{info, error, warn};
use core::cell::RefCell;
use embedded_sdmmc::{Block, BlockCount, BlockIdx, TimeSource, Timestamp};
use stm32h7xx_hal::hal::digital::v2::{InputPin, OutputPin, ToggleableOutputPin};
use stm32h7xx_hal::{gpio::*, pac, prelude::*, rcc, rcc::rec, rtc, sdmmc};

pub type SdFilesystem<'a> = embedded_sdmmc::Controller<SdioBlockDevice<'a>, DummyTimeSource>;

pub type SdFilesystemError = embedded_sdmmc::Error<sdmmc::Error>;

pub struct SdioBlockDevice<'a>(pub &'a RefCell<Sdmmc>);

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
        if let Err(e) = self.sdmmc.init_card(50.mhz()) {
            error!("init_card error {:?}", e);
        }
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

impl<'a> embedded_sdmmc::BlockDevice for SdioBlockDevice<'a> {
    type Error = sdmmc::Error;

    fn read(
        &self,
        blocks: &mut [Block],
        start_block_idx: BlockIdx,
        _reason: &str,
    ) -> Result<(), Self::Error> {
        let mut s = self.0.borrow_mut();
        let blocks: &mut [u8] = unsafe {
            core::slice::from_raw_parts_mut(
                blocks.as_mut_ptr() as *mut u8,
                blocks.len() * Block::LEN,
            )
        };
        s.led.set_high().unwrap();
        s.sdmmc.read_blocks(start_block_idx.0, blocks)?;
        s.led.set_low().unwrap();
        Ok(())
    }

    fn write(&self, blocks: &[Block], start_block_idx: BlockIdx) -> Result<(), Self::Error> {
        let mut s = self.0.borrow_mut();
        let mut start_idx = start_block_idx.0;

        for block in blocks {
            s.led.toggle().unwrap();
            s.sdmmc.write_block(start_idx, block)?;
            start_idx += Block::LEN as u32;
        }

        s.led.set_low().unwrap();

        Ok(())
    }

    fn num_blocks(&self) -> Result<BlockCount, Self::Error> {
        let s = self.0.borrow();
        Ok(BlockCount(s.sdmmc.card()?.size() as u32))
    }
}

pub struct DummyTimeSource;

impl TimeSource for DummyTimeSource {
    fn get_timestamp(&self) -> Timestamp {
        Timestamp {
            year_since_1970: 0,
            zero_indexed_month: 0,
            zero_indexed_day: 0,
            hours: 0,
            minutes: 0,
            seconds: 0,
        }
    }
}
