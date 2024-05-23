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
use networking::network_interface::NetworkHandler;
use networking::data_handler::DataHandler;

fn main() {
    
    // // define server socket connection
    let socket = SocketAddrV4::new(Ipv4Addr::new(172,20,10, 2), 8001);

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

    for data in receiver
    {
        println!("{:?}", data.return_joystick_data());
    }

    handle.join().unwrap();

}
