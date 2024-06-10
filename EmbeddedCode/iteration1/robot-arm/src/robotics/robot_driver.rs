/*
William Albertini

This module uses the high level SPI driver to 
send data to the motor controllers. Three Spi
structs are created as only one can handle a
single slave select (SS) line at a given time. Data
to be sent out on the bus is a u16 and must be
decoded before transmission (only one byte can
be sent at a time). This Spi uses CPOL=0 and 
CPHA=0, which is what the Atmega328p is expecting.
Bytes are written individually, allowing SS line
to return high between bytes (trigerring serial transfer
complete interrupt SPI STC)

*/




use rppal::spi::{Bus, Mode, Segment, SlaveSelect, Spi};


pub struct RobotDriver
{
	mac1: Spi,
	mac2: Spi,
	mac3: Spi,
}


impl RobotDriver
{
	pub fn new() -> RobotDriver
	{
		let mac1 = Spi::new(Bus::Spi1, SlaveSelect::Ss0, 8_000_000, Mode::Mode0).unwrap();
		let mac2 = Spi::new(Bus::Spi1, SlaveSelect::Ss1, 8_000_000, Mode::Mode0).unwrap();
		let mac3 = Spi::new(Bus::Spi1, SlaveSelect::Ss2, 8_000_000, Mode::Mode0).unwrap();

		RobotDriver{ mac1, mac2, mac3 }
	}

	pub fn write_mac(&mut self, data: u16, motor: u8, mac_number: u8)
	{
		let send_data = data_decomposition(data, motor);

		let _ = match mac_number
		{
			1 => {
				self.mac1.write(&[send_data[0]]).unwrap();
				std::thread::sleep(std::time::Duration::from_millis(2));
				self.mac1.write(&[send_data[1]])
				},
			2 => {
				self.mac2.write(&[send_data[0]]).unwrap();
				std::thread::sleep(std::time::Duration::from_millis(2));
				self.mac2.write(&[send_data[1]])
				},
			3 => {
				self.mac3.write(&[send_data[0]]).unwrap();
				std::thread::sleep(std::time::Duration::from_millis(2));
				self.mac3.write(&[send_data[1]])
				},
			_ => {
				println!("wrong motor selected");
				Ok(0)
			},
		};
		
	}

}

fn data_decomposition(data: u16, motor: u8) -> [u8; 2]
	{	
		// motor change value will be at largest 13 bits
		// two bits will be used on the most significant word (MSW)
		// to indicate byte and motor
		let msw = (motor << 6) | (1 << 7) | ((data >> 7) as u8);
		// grab least significant word, add 0 to begining to indicate LSW
		let lsw = (data & 0b1111111) as u8;

		[msw, lsw]
	}





#[cfg(test)]
mod tests {
	use super::*;

	#[test]
	fn test_data_decomposition_8() 
	{
		let send_val = data_decomposition(8, 1);

		assert_eq!([0b11000000, 0b1000], send_val);

	}

	#[test]
	fn test_data_decomposition_255()
	{
		let send_val = data_decomposition(255, 1);

		assert_eq!([0b11000001, 0b1111111], send_val);
	}

	#[test]
	fn test_data_decomposition_5000()
	{
		let send_val = data_decomposition(5000, 1);

		assert_eq!([0b11100111 ,0b00001000], send_val);
	}

	// #[test]
	// fn test_data_decomposition_5000_0()
	// {
	// 	let send_val = data_decomposition(5000, 0);

	// 	assert_eq!([0b00010011 ,0b10001000], send_val);
	// }
}


