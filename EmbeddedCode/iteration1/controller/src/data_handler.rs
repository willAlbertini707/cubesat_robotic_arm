/*
William Albertini

This module is built to make sending the data easier.
It provides a single struct that contains all user
input. This can be sent via UDP/IP to the
raspberry pi.
*/


pub struct DataHandler
{
    x: u8,
    y: u8,
    pitch: u8,
    roll: u8,
    button1: u8,
    button2: u8,
}

impl DataHandler 
{

    pub fn new() -> DataHandler 
    {
        DataHandler{
            x: 0,
            y: 0,
            roll: 0,
            pitch: 0,
            button1: 1,
            button2: 1,
        }
    }

    pub fn update(&mut self, joystick1: [u8; 3], joystick2: [u8; 3]) 
    {
        // unpack joystick 1
        self.x = joystick1[0];
        self.y = joystick1[1];
        self.button1 = joystick1[2];

        // unpack joystick 2
        self.roll = joystick2[0];
        self.pitch = joystick2[1];
        self.button2 = joystick2[2]
    }

    pub fn input_detected(&self) -> bool 
    {
        // checks to see if user input has been given
        if  self.x != 0 || 
            self.y != 0 || 
            self.button1 == 0 ||
            self.pitch != 0 ||
            self.roll != 0 ||
            self.button2 == 0
        {
            return true
        }
        false
    }

    pub fn data_as_bytes(&self) -> [u8; 6]
    {
        // input values are sent over udp and always in known order
        // for each individual datagram
        [self.x, self.y, self.roll, self.pitch,
        self.button1, self.button2]
    }
}