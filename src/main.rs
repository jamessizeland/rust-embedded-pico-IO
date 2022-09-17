//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]
// https://docs.rust-embedded.org/discovery/microbit/04-meet-your-hardware/terminology.html

mod pico_graphics;

use arrform::{arrform, ArrForm}; // embedded alternative to format!()
use bsp::entry;
use cortex_m::delay::Delay;
use defmt::*;
use defmt_rtt as _;
use embedded_hal::{
    adc::OneShot,
    digital::v2::{InputPin, ToggleableOutputPin},
};
use embedded_time::{fixed_point::FixedPoint, rate::Extensions};
use panic_probe as _;
use rp_pico as bsp; // Provide an alias for our BSP so we can switch targets quickly.
use usb_device::{class_prelude::*, prelude::*}; // USB Device support
use usbd_serial::SerialPort; // USB Communications Class Device support

use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    gpio,
    multicore::{Multicore, Stack},
    pac,
    sio::Sio,
    spi::Spi,
    usb,
    watchdog::Watchdog,
    Adc, Timer,
};

//**************SETUP DISPLAY**********/
use crate::pico_graphics::pico_display::PicoDisplay;
use display_interface_spi::SPIInterfaceNoCS;
use embedded_graphics::{
    draw_target::DrawTarget,
    image::Image,
    mono_font::{ascii::FONT_6X10, MonoTextStyle},
    prelude::Point,
    primitives::{Primitive, PrimitiveStyleBuilder, Rectangle},
    text::Text,
    transform::Transform,
};
use embedded_graphics::{image::ImageRawLE, pixelcolor::*};
use embedded_graphics::{
    prelude::{RgbColor, Size},
    Drawable,
};
use mipidsi::{models::*, NoPin, Orientation};
use mipidsi::{Display, DisplayOptions};
type DisplayRgb = Rgb565;

//**************Multi-core*************/
/// External high-speed crystal on the Raspberry Pi Pico board is 12 MHz. Adjust
/// if your board has a different frequency
const XTAL_FREQ_HZ: u32 = 12_000_000u32;

/// The frequency at which core 0 will blink its LED (Hz).
// const CORE0_FREQ: u32 = 3;
/// The frequency at which core 1 will blink its LED (Hz).
const CORE1_FREQ: u32 = 4;
/// The delay between each toggle of core 0's LED (us).
// const CORE0_DELAY: u32 = 1_000_000 / CORE0_FREQ;
/// The delay between each toggle of core 1's LED (us).
const CORE1_DELAY: u32 = 1_000_000 / CORE1_FREQ;
/// Stack for core 1
static mut CORE1_STACK: Stack<4096> = Stack::new();

//**************START MAIN******************/
#[entry]
fn main() -> ! {
    info!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);

    //********SETUP CLOCKS***********/
    let clocks = init_clocks_and_plls(
        XTAL_FREQ_HZ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();
    let timer = Timer::new(pac.TIMER, &mut pac.RESETS);
    let sys_freq = clocks.system_clock.freq().integer();
    let mut delay = Delay::new(core.SYST, sys_freq);

    //****************SETUP USB BUS***************/
    let usb_bus = UsbBusAllocator::new(usb::UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));

    // Set up the USB Communications Class Device driver
    let mut serial = SerialPort::new(&usb_bus);

    // Create a USB device with a fake VID and PID
    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27dd))
        .manufacturer("Fake company")
        .product("Serial port")
        .serial_number("TEST")
        .device_class(2) // from: https://www.usb.org/defined-class-codes
        .build();

    //**************SET UP GPIO PINS**********/
    let mut sio = Sio::new(pac.SIO);
    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );
    let mut led_pin = pins.led.into_push_pull_output();
    // let mut adc_pin_0 = pins.gpio26.into_floating_input();
    let sw = (
        pins.gpio12.into_pull_up_input(),
        pins.gpio13.into_pull_up_input(),
        pins.gpio14.into_pull_up_input(),
        pins.gpio15.into_pull_up_input(),
    );
    // Enable ADC
    let mut adc = Adc::new(pac.ADC, &mut pac.RESETS);
    let mut temperature_sensor = adc.enable_temp_sensor();
    //***************SET UP DISPLAY*************/
    // INIT SPI PINS TO BE USED UNDER THE HOOD BY DISPLAY
    // These are implicitly used by the spi driver if they are in the correct mode
    let _spi_sclk = pins.gpio18.into_mode::<gpio::FunctionSpi>();
    let _spi_mosi = pins.gpio19.into_mode::<gpio::FunctionSpi>();
    let spi_dc = pins.gpio16.into_push_pull_output();
    let _spi_cs = pins.gpio17.into_mode::<gpio::FunctionSpi>();
    let spi = Spi::<_, _, 8>::new(pac.SPI0);

    // Exchange the uninitialised SPI driver for an initialised one
    let spi = spi.init(
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        16_000_000u32.Hz(),
        &embedded_hal::spi::MODE_0,
    );

    //***************SET UP CORES*************/
    // CORE 1
    let mut mc = Multicore::new(&mut pac.PSM, &mut pac.PPB, &mut sio.fifo);
    let core1 = &mut mc.cores()[1];
    core1
        .spawn(unsafe { &mut CORE1_STACK.mem }, move || {
            // Get the second core's copy of the `CorePeripherals`, which are per-core.
            // Unfortunately, `cortex-m` doesn't support this properly right now,
            // so we have to use `steal`.
            let core = unsafe { pac::CorePeripherals::steal() };
            // Set up the delay for the second core.
            let mut delay = Delay::new(core.SYST, sys_freq);
            let di = SPIInterfaceNoCS::new(spi, spi_dc);
            const ORIGIN: Point = Point { x: 53, y: 40 }; //NOTE origin offset for Pico Display
            let mut display: Display<SPIInterfaceNoCS<Spi<_, _, 8_u8>, _>, NoPin, _> =
                Display::with_model(di, None, PicoDisplay::new());
            display.init(&mut delay, DisplayOptions::default()).unwrap();
            // clear the display to black
            display.clear(DisplayRgb::BLACK).unwrap();
            display
                .set_orientation(Orientation::Portrait(false))
                .unwrap();
            let raw_image_data = ImageRawLE::new(include_bytes!("../assets/ferris.raw"), 86);

            Image::new(&raw_image_data, Point::new(ORIGIN.x, ORIGIN.y))
                .translate(Point { x: 22, y: 15 })
                .draw(&mut display)
                .unwrap();
            // Blink the second LED.
            let style = PrimitiveStyleBuilder::new()
                .stroke_color(Rgb565::CSS_DARK_RED)
                .stroke_width(3)
                .fill_color(Rgb565::CSS_CRIMSON)
                .build();
            Rectangle::new(Point::new(ORIGIN.x, ORIGIN.y), Size::new(135, 240 - 90))
                .translate(Point { x: 0, y: 90 })
                .into_styled(style)
                .draw(&mut display)
                .unwrap();

            let mut style = MonoTextStyle::new(&FONT_6X10, Rgb565::WHITE);
            style.background_color = Some(Rgb565::CSS_CRIMSON);
            Text::new("Hello from Rust!", Point::new(ORIGIN.x, ORIGIN.y), style)
                .translate(Point { x: 5, y: 110 })
                .draw(&mut display)
                .unwrap();
            fn to_int(b: bool) -> u16 {
                if b {
                    1
                } else {
                    0
                }
            }
            loop {
                let btns: [u16; 4] = [
                    to_int(sw.0.is_low().unwrap()),
                    to_int(sw.1.is_low().unwrap()),
                    to_int(sw.2.is_low().unwrap()),
                    to_int(sw.3.is_low().unwrap()),
                ];
                led_pin.toggle().unwrap();
                let mut buf2 = [0u8; 32];
                let text = format_no_std::show(
                    &mut buf2,
                    format_args!("A:{}, B:{}, X:{}, Y:{}", btns[0], btns[1], btns[2], btns[3]),
                )
                .unwrap();
                Text::new(text, Point::new(ORIGIN.x, ORIGIN.y), style)
                    .translate(Point { x: 5, y: 120 })
                    .draw(&mut display)
                    .unwrap();
                delay.delay_us(CORE1_DELAY)
            }
        })
        .unwrap();
    let mut said_hello = false;
    delay.delay_ms(15000);
    // let mut iteration: usize = 0;
    loop {
        // iteration += 1;
        if !usb_dev.poll(&mut [&mut serial]) {
            continue;
        }
        // A welcome message at the beginning
        if !said_hello && timer.get_counter() >= 2_000_000 {
            said_hello = true;
            let _ = serial.write(b"Hello, World!\r\n");
        };
        // Read the raw ADC counts from the temperature sensor channel.
        let temp_sens_adc_counts: u16 = adc.read(&mut temperature_sensor).unwrap();
        // let pin_adc_counts: u16 = adc.read(&mut adc_pin_0).unwrap();
        let mut buf = [0u8; 32];
        arrform!(
            64,
            "T:{} {}\r\n",
            temp_sens_adc_counts,
            cal_temp(temp_sens_adc_counts),
        )
        .as_bytes()
        .iter()
        .take(32)
        .enumerate()
        .for_each(|(i, b)| {
            buf[i] = *b;
        });

        let mut wr_ptr = &buf[..buf.len()];
        // count bytes were written
        while !wr_ptr.is_empty() {
            match serial.write(wr_ptr) {
                Ok(len) => wr_ptr = &wr_ptr[len..],
                // On error, just drop unwritten data.
                // One possible error is Err(WouldBlock), meaning the USB
                // write buffer is full.
                Err(_) => break,
            };
        }
    }
}

fn cal_temp(adc_count: u16) -> u16 {
    // T = 27 - (ADC_voltage - 0.706)/0.001721
    let voltage: f32 = (adc_count as f32) * 0.000805664; // 12bit adc at 3.3v
    let temperature = 27.0 - (voltage - 0.706) / 0.001721;
    temperature as u16
}

// End of file
