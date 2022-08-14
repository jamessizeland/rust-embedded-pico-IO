use bsp::{
    hal::{
        adc,
        gpio::{Pin, PinId, PinMode, ValidPinMode},
        sio::Sio,
        Adc,
    },
    pac::Peripherals,
    Pins,
};
use embedded_hal::adc::OneShot;
use rp_pico as bsp; // Provide an alias for our BSP so we can switch targets quickly.

pub struct PicoGPIO {
    adc: Adc,
    sensor: adc::TempSense,
    pins: Pins,
}

impl PicoGPIO {
    pub fn new(pac: Peripherals, sio: Sio) -> Self {
        let adc_temp = Adc::new(pac.ADC, &mut pac.RESETS);
        // let sio_temp = Sio::new(pac.SIO);
        PicoGPIO {
            adc: adc_temp,
            sensor: adc_temp.enable_temp_sensor(),
            pins: Pins::new(
                pac.IO_BANK0,
                pac.PADS_BANK0,
                sio.gpio_bank0,
                &mut pac.RESETS,
            ),
        }
    }
    /// Convert the Pico onboard temperature sensor from ADC Counts to degC
    pub fn read_tc(&self) -> u16 {
        // Read the raw ADC counts from the temperature sensor channel.
        let temp_sens_adc_counts: u16 = self.adc.read(&mut self.sensor).unwrap();
        temp_sens_adc_counts
    }
    /// Read an ADC value directly from hardware
    // pub fn read_adc<I, M>(&self, pin: &mut Pin<I, M>) -> u16
    // where
    //     I: PinId,
    //     M: PinMode + ValidPinMode<I>,
    // {
    //     let pin_adc_counts: u16 = self.adc.read(pin).unwrap();
    //     pin_adc_counts
    // }
    /// Get the pin struct to initialize pins
    pub fn get_pin(&self) -> Pins {
        self.pins
    }
}
