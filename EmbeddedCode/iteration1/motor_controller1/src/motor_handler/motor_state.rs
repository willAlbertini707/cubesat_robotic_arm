/*
William Albertini

This module controls the logic for the
motor state machine. The transfer from one
state to another either decrements or
imcrements a count. Transition states are 
pre-defined and used to determine the direction 
the motor is turning. Each encoder has a 
specific number of encoder ticks per 
revolution, and this information is used
by the inverse kinematic solver (on the RPI) 
to determine motor position.

*/



// constants for comparing state transitions
// (past state)_(new state)
const STATE1_STATE2: i8 = 0b0001;
const STATE1_STATE4: i8 = 0b0010;
const STATE2_STATE3: i8 = 0b0111;
const STATE2_STATE1: i8 = 0b0100;
const STATE3_STATE4: i8 = 0b1110;
const STATE3_STATE2: i8 = 0b1101;
const STATE4_STATE1: i8 = 0b1000;
const STATE4_STATE3: i8 = 0b1011;


#[derive(Debug, Clone, Copy)]
pub enum MotorState {
    // current state of motor
    State1 = 0b00,
    State2 = 0b01,
    State3 = 0b11,
    State4 = 0b10,
}

impl MotorState{

    fn new(a: bool, b: bool) -> MotorState {
        
        // check current pins and configures state
        if !a && !b {
            return MotorState::State1
        } else if !a && b {
            return MotorState::State2
        } else if a && b {
            return MotorState::State3
        } else {
            return MotorState::State4
        }
    }
}

#[derive(Debug)]
pub struct Motor {
    pub state: MotorState,
    position: i16,
    ticks_per_rev: i16,
}


impl Motor {
    
    pub fn new(a: bool, b: bool, ticks_per_rev: i16) -> Motor {
        
        let motor_state = MotorState::new(a, b);

        // set default state to current state and position = 0
        Motor {
            state: motor_state,
            position: 0,
            ticks_per_rev,
        }
    }

    pub fn set_motor_state(&mut self, a: bool, b: bool) {
        let motor_state = MotorState::new(a, b);

        self.state = motor_state;
    }

    pub fn update_motor_state(&mut self, a: bool, b: bool){
        // pre-calculate next state
        let new_state = MotorState::new(a, b);

        // get combination of past and current
        // create bits to map to state change
        let combo_state: i8 = ((self.state as i8) << 2) | (new_state as i8);

        // determine the transition of the state and update position
        match combo_state {
            STATE1_STATE2 => self.position += 1,
            STATE1_STATE4 => self.position -= 1,
            STATE2_STATE3 => self.position += 1,
            STATE2_STATE1 => self.position -= 1,
            STATE3_STATE4 => self.position += 1,
            STATE3_STATE2 => self.position -= 1,
            STATE4_STATE1 => self.position += 1,
            STATE4_STATE3 => self.position -= 1,
            _ => self.state = MotorState::new(a,b),
        }


        // update state
        self.state = new_state;
    }

    pub fn get_position(&self) -> i16 {
        // returns current position
        self.position
    }
}