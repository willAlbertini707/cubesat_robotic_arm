/*
William Albertini

This module represents the end effector state. It will
be used to keep track of the eff state depending on the 
input from the controller. 
*/

use crate::network_interface::DataHandler;

struct EndEffectorPosition {
    x: i16,
    y: i16,
    roll: i16,
    pitch: i16,
}

impl EndEffectorPosition {

    pub fn update_from_data_handler(data: DataHandler)
    {
        9;
    }
}