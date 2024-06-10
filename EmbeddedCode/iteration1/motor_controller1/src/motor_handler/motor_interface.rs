/*
William Albertini

MotorInterface acts as a high-level driver for 
motor actuation and state determination. The
driver uses in(1 or 2) pins to switch the polarity
of the DC motor leads. The enable pin determines
how much voltage the motor sees. The quadrature
encoder states are updated upon a rising or falling
edge to the selected input pins. MotorInterface uses
a MotorState to keep track of position and state.

*/



// external imports
use arduino_hal::port::{
    mode::Output, 
    mode::Input,
    mode::Floating, 
    Pin,
    PinOps,
    mode::PwmOutput,
};
use arduino_hal::simple_pwm::PwmPinOps;

// internal imports
use super::motor_state::Motor;

// struct for controlling motor interface
pub struct MotorInterface <T, U, W, Y, X, Z> 
where
    T: PinOps,
    U: PinOps,
    W: PwmPinOps<Y>,
    X: PinOps,
    Z: PinOps,
{
    in1: Pin<Output, T>,
    in2: Pin<Output, U>,
    en: Pin<PwmOutput<Y>, W>,
    a: Pin<Input<Floating>, X>,
    b: Pin<Input<Floating>, Z>,
    state: Motor,
}


impl<T, U, W, Y, X, Z>  MotorInterface<T, U, W, Y, X, Z> 
where
    T: PinOps,
    U: PinOps,
    W: PwmPinOps<Y>,
    X: PinOps,
    Z: PinOps,
{
    pub fn new(in1: Pin<Output, T>,
               in2: Pin<Output, U>, 
               mut en: Pin<PwmOutput<Y>, W>,
               a: Pin<Input<Floating>, X>,
               b: Pin<Input<Floating>, Z>) -> MotorInterface<T, U, W, Y, X, Z> 
    {
        en.enable();
        // set initial motor state
        let motor = Motor::new(a.is_high(), b.is_high(), 8000);
        
        // return struct
        MotorInterface{
            in1,
            in2,
            en,
            a,
            b,
            state: motor,
        }
    }

    pub fn turn_to_position(&mut self, new_position: i16) -> bool 
    {

        // figure out it a CW or CCW operation is needed
        let position_error = new_position - self.state.get_position();
        
        // check whether to turn CW or CCW
        if position_error > 0 
        {
            self.in1.set_high();
            self.in2.set_low();
        } 
        else if position_error < 0
        {
            self.in1.set_low();
            self.in2.set_high();
        }
        else 
        {
            // motor is at desired position
            self.en.set_duty(0);
            return true
        }

        self.en.set_duty(80);
        return false
    }

    pub fn update_position(&mut self) 
    {
        // update the state of the motor
        self.state.update_motor_state(self.a.is_high(), self.b.is_high());
    }

    pub fn get_position(&self) -> i16 
    {
        // return the position of the motor
        self.state.get_position()
    }
}

