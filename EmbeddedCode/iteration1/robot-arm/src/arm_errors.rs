/*
William Albertini

Custom error enum for this project.

*/


use std::fmt;


#[derive(Debug, PartialEq)]
pub enum RoboticArmError {
	Singularity(String),
	NetworkError(String),
	BadPipe(String),
	KinematicJointsNotUpdated(String)

}


impl fmt::Display for RoboticArmError 
{
	fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result 
	{
		match &self 
		{
			self::RoboticArmError::Singularity(em) => write!(f,
				"{em}"),
			self::RoboticArmError::NetworkError(em) => write!(f,
				"{}", em),
			self::RoboticArmError::BadPipe(em) => write!(f,
				"{}", em),
			self::RoboticArmError::KinematicJointsNotUpdated(em) => write!(f,
				"{}", em),
		}
	}
}