/*
William Albertini

This module is built to make sending the data easier.
It provides a single struct that contains all user
input. This can serialized and sent via TCP/IP to 
raspberry pi.
*/

#[derive(Clone, Copy)]
pub struct DataHandler
{
    pub x: i16,
    pub y: i16,
    pub roll: i16,
    pub pitch: i16,
    button1: i16,
    button2: i16,
}

impl DataHandler 
{
    pub fn from_buffer(buffer: &[i16; 6]) -> DataHandler 
    {
        DataHandler{
            x: buffer[0],
            y: buffer[1],
            roll: buffer[2],
            pitch: buffer[3],
            button1: buffer[4],
            button2: buffer[5],
        }
    }

    pub fn return_joystick_data(&self) -> ([i16; 3], [i16; 3])
    {
        (
            // data from first joystick
            [self.x, self.y, self.button1],
            // data from second joystick
            [self.roll, self.pitch, self.button2]
        )
    }

    pub fn both_buttons_pressed(&self) -> bool
    {
        (self.button1 == 0) && (self.button2 == 0)
    }


}