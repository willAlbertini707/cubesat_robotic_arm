#![no_std]
#![no_main]
#![feature(abi_avr_interrupt)]

// external imports
use panic_halt as _;
use ufmt;
use core::sync::atomic::{AtomicBool, Ordering};
use arduino_hal::port::{Pin, mode::Input, mode::Floating};
use arduino_hal::simple_pwm::{Timer0Pwm, IntoPwmPin, *};

// internal imports
// mod motor_state;
mod motor_handler;
use motor_handler::motor_interface::MotorInterface;

// flag to detect interrupt
static TRIGGERED: AtomicBool = AtomicBool::new(false);

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


    loop {
        // check if interrupt was triggered
        if TRIGGERED.load(Ordering::SeqCst) {
            TRIGGERED.store(false, Ordering::SeqCst);
            motor_one.update_position();
            // ufmt::uwriteln!(serial, "Motor position: {}", motor_one.get_position()).unwrap();
        }

        motor_one.turn_to_position(10);
    }
}