#![no_std]
#![no_main]
#![feature(abi_avr_interrupt)]

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

#[avr_device::interrupt(atmega328p)]
#[allow(non_snake_case)]
fn SPI_STC() {
    READ_SPI_REGISTER.store(true, Ordering::SeqCst);
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

    // let pal = pins.d6.into_output().into_pwm(&timer0);

    // enable interrupts on PCIE2 (0b100)
    dp.EXINT.pcicr.write(|w| unsafe {w.bits(0b100)});

    // enable interrupts on PCINT18 (d2) and PCINT19 (d3)
    dp.EXINT.pcmsk2.write(|w| unsafe {w.bits(0b1100)});

    // global enable interrupts
    unsafe {
        avr_device::interrupt::enable();
    }

    let mut pos: i16 = 4500;

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
            pos = unsafe {spi_read_reg_ptr.read()} as i16;
            // ufmt::uwriteln!(serial, "Position: {}", pos);
        }

        motor_one.turn_to_position(pos);
    }
}


fn set_up_spi_slave_mode(dp: &Peripherals)
{
    dp.PORTB.ddrb.write(|w| unsafe {w.bits(0b10000)});
    dp.SPI.spcr.write(|w| unsafe {w.bits(0b11000000)});
}