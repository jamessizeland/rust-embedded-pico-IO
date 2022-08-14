use display_interface::{DataFormat, DisplayError, WriteOnlyDataCommand};
// use embedded_graphics_core::{pixelcolor::Rgb565, prelude::IntoStorage};
use embedded_graphics::pixelcolor;
use embedded_graphics::prelude::IntoStorage;
use embedded_hal::{blocking::delay::DelayUs, digital::v2::OutputPin};

// use mipidsi::no_pin::NoPin;
use mipidsi::{instruction::Instruction, Error};
use mipidsi::{DisplayOptions, Orientation};

use super::{write_command, Model};

/// ST7789 SPI display with Reset pin
/// Only SPI with DC pin interface is supported
pub struct PicoDisplay;

impl Model for PicoDisplay {
    type ColorFormat = pixelcolor::Rgb565;
    // type ColorFormat = pixelcolor::Bgr565;

    fn new() -> Self {
        Self
    }

    fn init<RST, DELAY, DI>(
        &mut self,
        di: &mut DI,
        rst: &mut Option<RST>,
        delay: &mut DELAY,
        options: DisplayOptions,
    ) -> Result<u8, Error<RST::Error>>
    where
        RST: OutputPin,
        DELAY: DelayUs<u32>,
        DI: WriteOnlyDataCommand,
    {
        // let madctl = options.madctl() ^ 0b0000_1000; // this model has flipped RGB/BGR bit
        let madctl: u8 = options.madctl();
        match rst {
            Some(ref mut rst) => self.hard_reset(rst, delay)?,
            None => write_command(di, Instruction::SWRESET, &[])?,
        }
        delay.delay_us(150_000);

        write_command(di, Instruction::SLPOUT, &[])?; // turn off sleep
        delay.delay_us(10_000);

        write_command(di, Instruction::INVOFF, &[])?;
        write_command(di, Instruction::VSCRDER, &[0u8, 0u8, 0x14u8, 0u8, 0u8, 0u8])?;
        write_command(di, Instruction::MADCTL, &[madctl])?; // left -> right, bottom -> top RGB

        write_command(di, Instruction::COLMOD, &[0b0101_0101])?; // 16bit 65k colors
        write_command(di, Instruction::INVON, &[])?;
        delay.delay_us(10_000);
        write_command(di, Instruction::NORON, &[])?; // turn to normal mode
        delay.delay_us(10_000);
        write_command(di, Instruction::DISPON, &[])?; // turn on display

        // DISPON requires some time otherwise we risk SPI data issues
        delay.delay_us(120_000);

        Ok(madctl)
    }

    fn write_pixels<DI, I>(&mut self, di: &mut DI, colors: I) -> Result<(), DisplayError>
    where
        DI: WriteOnlyDataCommand,
        I: IntoIterator<Item = Self::ColorFormat>,
    {
        write_command(di, Instruction::RAMWR, &[])?;
        let mut iter = colors.into_iter().map(|c| c.into_storage());

        let buf = DataFormat::U16BEIter(&mut iter);
        di.send_data(buf)
    }

    fn display_size(&self, _orientation: Orientation) -> (u16, u16) {
        (135, 240)
    }

    fn framebuffer_size(&self, orientation: Orientation) -> (u16, u16) {
        match orientation {
            // Orientation::Portrait(_) | Orientation::PortraitInverted(_) => (240, 320),
            // Orientation::Landscape(_) | Orientation::LandscapeInverted(_) => (320, 240),
            Orientation::Portrait(_) | Orientation::PortraitInverted(_) => (240, 320),
            Orientation::Landscape(_) | Orientation::LandscapeInverted(_) => (320, 240),
        }
    }
}
