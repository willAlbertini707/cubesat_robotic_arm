/*
William Albertini

this module handle the networking interface
for the pi and the esp32. It uses a Udp socket
and holds open a server indefinitely. Udp was found 
to perform much better than Tcp in terms of real-time
data transmission. 

NetworkHandler a high level wrapper for Udp sockets and
maintains a piped connection between the server thread
and the motor control thread. A new NetWorkHandler needs
the IPV4 socket that the ESP32 is looking for. In 
launch_server(), a mspc Sender is used to pipe data 
to a queue that the main thread reads. 

*/

use std::net::{
	UdpSocket,
	SocketAddrV4,
};
use std::sync::mpsc::Sender;
// internal imports
use crate::arm_errors::RoboticArmError;
use super::data_handler::DataHandler;


// create struct handle network interfacing
pub struct NetworkHandler
{
	socket: SocketAddrV4,
}

impl NetworkHandler 
{

	pub fn new(socket: SocketAddrV4) -> NetworkHandler 
	{
		NetworkHandler{socket}
	}

	pub fn launch_server(&mut self, sender: Sender<DataHandler>) -> Result<(), RoboticArmError> 
	{
		// try to bind ipv4 socket to tcp listener
		let socket = match UdpSocket::bind(self.socket)
		{
			Ok(socket) => socket,
			Err(_) => return Err(RoboticArmError::NetworkError("Failure to bind to socket".into())),
		};
		println!("Running on port: {:?}", self.socket);

		// launch server and wait for connections
		let mut buffer: [u8; 6] = [0; 6];
		loop 
		{
			// (amoumt, source) amount is a fixed size [u8; 0] and no messages go back to source
			let (_, _) = socket.recv_from(&mut buffer).expect("nothing");
			// pipe data back to main thread
			sender.send(self.process_buffer(&buffer)).expect("Bad pipe");
		}

		#[allow(unreachable_code)]
		Ok(())
	}

	

	fn process_buffer(&self, buffer: &[u8; 6]) -> DataHandler
	{
	// buffer is received as [x, y, roll, pitch, button1, button2]
		// create empty buffer
		let mut buffer_i16: [i16; 6] = [0; 6];
		// data is received in a single packet with known order
		for i in 0..=5
		{
			if buffer[i] > 8
			{
				buffer_i16[i] = -((buffer[i] >> 4) as i16);
			} else {
				buffer_i16[i] = buffer[i] as i16;
			}
			// make values correct for stick layout (left and right stick are opposite directions)
			if i == 1 || i == 2
			{
				buffer_i16[i] *= -1;
			}
		}
		
		DataHandler::from_buffer(&buffer_i16)
	}
}