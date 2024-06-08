/*
William Albertini

This module handles the hardware interface. It makes sampling the 
joysticks more intuitive and less verbose in main.
*/
use esp_idf_svc::hal;
use hal::gpio::{
    Pin,
    PinDriver,
    ADCPin,
    Input,
};
use hal::adc::{
    AdcChannelDriver,
    attenuation,
    AdcDriver,
    ADC1,
};
use esp_idf_svc::hal::sys::EspError;

// create joystick object to handle interfacing
pub struct JoyStick<'a, T, U, V>
where
    T: ADCPin<Adc = ADC1>,
    V: ADCPin<Adc = ADC1>,
    U: Pin, 
{
    x: AdcChannelDriver<'a, {attenuation::DB_11}, T>,
    y: AdcChannelDriver<'a, {attenuation::DB_11}, V>,
    button: PinDriver<'a, U, Input>,
}


impl<'a, T, U, V> JoyStick<'a, T, U, V>
where
    T: ADCPin<Adc = ADC1>,
    V: ADCPin<Adc = ADC1>,
    U: Pin,
{
    pub fn new(
        x: AdcChannelDriver<'a, {attenuation::DB_11}, T>,
        y: AdcChannelDriver<'a, {attenuation::DB_11}, V>,
        button: PinDriver<'a, U, Input>,
    ) -> JoyStick<'a, T, U, V> 
    {
        JoyStick{x, y, button}
    }

    pub fn sample(&mut self, adc_access: &mut AdcDriver<ADC1>) -> anyhow::Result<[u8; 3], EspError>
    {
        // read values from all sensors
        let x_val: u16 = adc_access.read(&mut self.x)?;
        let y_val: u16 = adc_access.read(&mut self.y)?;
        let button_val: u8 = self.button.is_high() as u8;

        // return values in array
        Ok([self.scale_value(x_val), 
            self.scale_value(y_val), 
            button_val])
    }

    fn scale_value(&self, value: u16) -> u8 
    {
        // scale values of sensor readings
        // this range marks no change
        if value > 1550 && value < 1750 
        {
            return 0 as u8
        } 
        else if value < 1550  // this indicates a left movement
        {
            return (8 - (value * 8 / 1550) ) as u8
        }

        else // this indicates a right movement (will be bit shifter back later to 0-8 range)
        {
            // perform scaling (values dont reach 2814 only 2765 for some reason)
            let mut value = (value * 8 / (2765 - 1750) - 13);
            if value > 8 {value = 8};

            return (value << 4) as u8
        }
            
    }
}