// external imports
use std::net::
{
    Ipv4Addr,
    SocketAddrV4,
};
use std::thread;
use std::sync::mpsc::{self, Receiver, Sender};
// internal imports
mod robotics;
mod arm_errors;
mod networking;
// mod hardware_interface;
use networking::network_interface::NetworkHandler;
use networking::data_handler::DataHandler;
use robotics::arm_state::{RoboticArmSolver, ArmState, AngleToEncoderMap};
use robotics::robot_driver::RobotDriver;

fn main() {
    
    // // define server socket connection
    let socket = SocketAddrV4::new(Ipv4Addr::new(172,20,10, 3), 8001);

    // network connection
    let mut network = NetworkHandler::new(socket);
    
    // define pipe
    let sender: Sender<DataHandler>;
    let receiver: Receiver<DataHandler>;
    (sender, receiver) = mpsc::channel();

    // launch server in a seperate thread
    let handle = thread::spawn(move || {
        network.launch_server(sender).unwrap();
    });

    // create a map (angular encoder ticks per revolution)
    let joint_map = AngleToEncoderMap::new(5000, 5000, 5000, 5000, 5000);
    let mut robotic_arm = RoboticArmSolver::try_new_from_ef_position(1000.0, 500.0, 300.0, 1800.0, 0.0, 0.0, joint_map).expect("Failed to construct arm");
    // create driver for interface
    let mut driver = RobotDriver::new();

    for data in receiver
    {
        println!("{:?}", data.return_joystick_data());
        
        // handle case of singularities or EF out of workspace
        if let Err(e) = robotic_arm.update_from_data_handler(data)
        {
            println!("Requested EF position unavailable");
        }

        let delta: ArmState = robotic_arm.get_delta_joints();
        println!("Delta joints: {:?}", delta);
        driver.write_mac(delta.roll, 1, 2);
    }

    handle.join().unwrap();

}
