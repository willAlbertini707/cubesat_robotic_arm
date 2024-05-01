#![no_std]
#![no_main]
#![feature(abi_avr_interrupt)]

// external imports
use panic_halt as _;
use ufmt;
use core::sync::atomic::{AtomicBool, Ordering};
use arduino_hal::hal::port::PD2;
use avr_device::interrupt::Mutex;
use arduino_hal::port::{Pin, mode::Input, mode::Floating};

// internal imports
mod motor_state;
use motor_state::Motor;

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

    // define quad encoder pins
    let a = pins.d2.into_floating_input();

    let b = pins.d3.into_floating_input();

    // enable interrupts on PCIE2 (0b100)
    dp.EXINT.pcicr.write(|w| unsafe {w.bits(0b100)});

    // enable interrupts on PCINT18 (d2) and PCINT19 (d3)
    dp.EXINT.pcmsk2.write(|w| unsafe {w.bits(0b1100)});

    // global enable interrupts
    unsafe {
        avr_device::interrupt::enable();
    }

    // create motor object
    let mut motor1 = Motor::new(a.is_high(), b.is_high(), 8000);

    loop {
        // check if interrupt was triggered
        if TRIGGERED.load(Ordering::SeqCst) {
            TRIGGERED.store(false, Ordering::SeqCst);
            let a_val = a.is_high();
            let b_val = b.is_high();

            motor1.update_motor_state(a_val, b_val);


            // ufmt::uwriteln!(serial, "position: {}", motor1.get_position()).unwrap();
            ufmt::uwriteln!(serial, "state: {}", motor1.state as i8).unwrap();
        }
    }
}