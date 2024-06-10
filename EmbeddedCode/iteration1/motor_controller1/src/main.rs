#![no_std]
#![no_main]
#![feature(abi_avr_interrupt)]

/*
William Albertini

This code (and associated modules) controls 
the motors, motor logic, and data receiving 
for the motor controller. Only flags are used
in ISRs which tell the main code to sample pins
or pull data from SPI data register. The motor 
logic is contained in motor_handler::motor_state,
and the driver for the motor is contained in 
motor_handler::motor_interface. Data received on the
SPI bus is seperated into two bytes. The decoding of 
this data is handled by decode_transmission() in this
module.

*/

// external imports
use panic_halt as _;
use core::sync::atomic::{AtomicBool, Ordering};
use arduino_hal::simple_pwm::{Timer0Pwm, IntoPwmPin, Prescaler};
use arduino_hal::Peripherals;

// internal imports
// mod motor_state;
mod motor_handler;
use motor_handler::motor_interface::MotorInterface;

// flag to detect interrupt
static TRIGGERED: AtomicBool = AtomicBool::new(false);
static READ_SPI_REGISTER: AtomicBool = AtomicBool::new(false);

// ISR for SPI end of transmission
#[avr_device::interrupt(atmega328p)]
#[allow(non_snake_case)]
fn SPI_STC() {
    avr_device::interrupt::free(|_cs| {
        READ_SPI_REGISTER.store(true, Ordering::SeqCst);
    })
}

// ISR for quad encoder pins
#[avr_device::interrupt(atmega328p)]
#[allow(non_snake_case)]
fn PCINT2() {
    avr_device::interrupt::free(|_cs| {
        TRIGGERED.store(true, Ordering::SeqCst);
    })
}

#[arduino_hal::entry]
fn main() -> ! {
    let dp = arduino_hal::Peripherals::take().unwrap();

    // setup spi and interrupts
    set_up_spi_slave_mode(&dp);
    let spi_read_reg_ptr: *mut u8 = dp.SPI.spdr.as_ptr();
    
    let pins = arduino_hal::pins!(dp);
    let mut serial = arduino_hal::default_serial!(dp, pins, 57600);

    // select timer for PWM
    let timer0 = Timer0Pwm::new(dp.TC0, Prescaler::Prescale64);
    
    // create motor object
    let mut motor_one = MotorInterface::new(
        pins.d4.into_output(),
        pins.d5.into_output(),
        pins.d6.into_output().into_pwm(&timer0),
        pins.d2.into_floating_input(),
        pins.d3.into_floating_input(),
    );

    // enable interrupts on PCIE2 (0b100)
    let pcie2: u8 = 1 << 2;
    dp.EXINT.pcicr.write(|w| unsafe {w.bits(pcie2)});

    // enable interrupts on PCINT18 (d2) and PCINT19 (d3)
    let pcint18: u8 = 1 << 2;
    let pcint19: u8 = 1 << 3;
    dp.EXINT.pcmsk2.write(|w| unsafe {w.bits(pcint18 | pcint19)});

    // global enable interrupts
    unsafe {
        avr_device::interrupt::enable();
    }

    // predefine data for motors and data decoding
    let mut pos: i16 = 4500;
    let mut data: u8 = 0;
    let mut add_lsw: bool = false;
    let mut motor_number: u8 = 0;
    let mut data_buffer: [u8; 2] = [0; 2];

    loop 
    {
        // check if interrupt was triggered
        if TRIGGERED.load(Ordering::SeqCst) {
            // clear flag
            TRIGGERED.store(false, Ordering::SeqCst);
            motor_one.update_position();
        }

        // check if data came from SPI
        if READ_SPI_REGISTER.load(Ordering::SeqCst) {
            // clear flag
            READ_SPI_REGISTER.store(false, Ordering::SeqCst);
            data = unsafe {spi_read_reg_ptr.read()};
            
            // handle data and determine MSW and LSW
            let read_buffer = decode_transmission(data, &mut data_buffer, &mut motor_number, &mut add_lsw);
            
            if read_buffer
            {
                // if data came in order, update that data
                pos = (((data_buffer[0] as u16) << 7 )| (data_buffer[1] as u16)) as i16;
                
                // print for debugging
                ufmt::uwriteln!(serial, "Position: {}", pos);
            }
        }

        motor_one.turn_to_position(pos);
    }
}


// enable MISO as output and 
// write SPI interrupt enable and SPI enable as 1
fn set_up_spi_slave_mode(dp: &Peripherals)
{
    // define MISO position
    let miso: u8 = 1 << 4;
    // define SPI and SPI interrupt enable
    let spie: u8 = 1 << 7;
    let spe: u8 = 1 << 6;

    // write to data direction register
    dp.PORTB.ddrb.write(|w| unsafe {w.bits(miso)});
    // write to spi control register
    dp.SPI.spcr.write(|w| unsafe {w.bits(spie | spe)});
}

// function to decode bytes obtained on spi data register
// data comes in the form of MSW (1_motor#_6bits) or
// LSW (0_7bits)
fn decode_transmission(data: u8, buffer: &mut [u8; 2], motor: &mut u8, add_lsw: &mut bool) -> bool
{
    // check if data received is MSW or LSW
    if data >> 7 == 1 
    {
        // grab the motor number
        *motor = (data >> 6) & 0b01;
        // store MSW in data buffer (6 bits)
        buffer[0] = data & 0b111111;
        // activate flag to fill rest of buffer (order must be MSW then LSW)
        *add_lsw = true;

    } else if (data >> 7) == 0 && *add_lsw {
        // add lsw to buffer
        buffer[1] = data;
        // reset flag
        *add_lsw = false;
        return true
    } else {
        *add_lsw = false;
    }
    false
}