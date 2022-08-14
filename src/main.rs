//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]
// https://docs.rust-embedded.org/discovery/microbit/04-meet-your-hardware/terminology.html

use arrform::{arrform, ArrForm}; // embedded alternative to format!()
use bsp::entry;
use cortex_m::delay::Delay;
use defmt::*;
use defmt_rtt as _;
use embedded_hal::{
    adc::OneShot,
    digital::v2::{InputPin, OutputPin, ToggleableOutputPin},
};
use embedded_time::fixed_point::FixedPoint;
use panic_probe as _;
use rp_pico as bsp; // Provide an alias for our BSP so we can switch targets quickly.
use usb_device::{class_prelude::*, prelude::*}; // USB Device support
use usbd_serial::SerialPort; // USB Communications Class Device support

use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    multicore::{Multicore, Stack},
    pac,
    sio::Sio,
    usb,
    watchdog::Watchdog,
    Adc, Timer,
};

//**************Multi-core****/
/// External high-speed crystal on the Raspberry Pi Pico board is 12 MHz. Adjust
/// if your board has a different frequency
const XTAL_FREQ_HZ: u32 = 12_000_000u32;

/// The frequency at which core 0 will blink its LED (Hz).
const CORE0_FREQ: u32 = 3;
/// The frequency at which core 1 will blink its LED (Hz).
const CORE1_FREQ: u32 = 4;
/// The delay between each toggle of core 0's LED (us).
const CORE0_DELAY: u32 = 1_000_000 / CORE0_FREQ;
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

    // Configure the clocks
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

    //****************SETUP USB BUS***************/
    // Set up the USB driver
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
    let mut adc_pin_0 = pins.gpio26.into_floating_input();
    let sw_a = pins.gpio12.into_pull_up_input();
    // Enable ADC
    let mut adc = Adc::new(pac.ADC, &mut pac.RESETS);
    let mut temperature_sensor = adc.enable_temp_sensor();

    //***************SET UP CORES*************/
    let sys_freq = clocks.system_clock.freq().integer();
    // CORE 0
    let mut delay = Delay::new(core.SYST, sys_freq);
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
            // Blink the second LED.
            loop {
                led_pin.toggle().unwrap();
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
        let pin_adc_counts: u16 = adc.read(&mut adc_pin_0).unwrap();
        let button_a: u16 = if sw_a.is_low().unwrap() { 1 } else { 0 };
        let mut buf = [0u8; 64];
        // Send back to the host
        // let mut wr_ptr = &buf[..64];
        arrform!(
            64,
            "Temp:{:02} Pin: {:02} A: {:02}\r\n",
            temp_sens_adc_counts,
            pin_adc_counts,
            button_a,
        )
        .as_bytes()
        .iter()
        .take(64)
        .enumerate()
        .for_each(|(i, b)| {
            buf[i] = *b;
        });
        let mut wr_ptr = &buf[..buf.len()];
        // count bytes were written
        while !wr_ptr.is_empty() {
            match serial.write(&wr_ptr) {
                Ok(len) => wr_ptr = &wr_ptr[len..],
                // On error, just drop unwritten data.
                // One possible error is Err(WouldBlock), meaning the USB
                // write buffer is full.
                Err(_) => break,
            };
        }
    }
}

// End of file
