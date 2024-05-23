/* 
William Albertini

The purpose of this module is to incorporate a motor controller
capable of calculating joint angles and writing those
to motor controllers
*/

// external imports
use std::f64::consts::PI;

// internal imports
use crate::arm_errors::RoboticArmError;

// contain joint angles, linkage lengths, and up/down elbow solver
pub struct RoboticArm 
{
	// joint and spool angles for robotic arm
	joint_one: i16,
	joint_two: i16,
	joint_three: i16,
	joint_four: i16,
	spool: i16,
	// linkage lengths
	link1: f64,
	link2: f64,
	link3: f64,
	// elbow up or down solution
	up: bool,
}


impl RoboticArm 
{
	pub fn new(link1: f64, link2: f64, link3: f64) -> RoboticArm 
	{
		RoboticArm
		{
			joint_one: 0,
			joint_two: 0,
			joint_three: 0,
			joint_four: 0,
			spool: 0,
			link1,
			link2,
			link3,
			up: false,
		}
	}

	pub fn max_end_effector_distance(&self) -> f64
	{
		self.link1 + self.link2 + self.link3
	}

	fn find_joint_angles(&self, x3: f64, y3: f64, si: f64) -> Result<[f64; 3], RoboticArmError>
	{

		// ----------------------------- theta 2 ----------------------------------------
		// calculate the location of the elbow joint
		let x2: f64 = x3 - self.link3 * si.cos();
		let y2: f64 = y3 - self.link3 * si.sin();


		// find the angular position of the elbow joint ( theta2 elbow up version)
		let mut theta2 = ((x2*x2 + y2*y2 - (self.link1*self.link1 + self.link2 * self.link2))
						 / (2.0*self.link1 * self.link2)).acos();

		if theta2.is_nan() 
		{
			return Err(RoboticArmError::Singularity("Singularity it theta 2".into()));
		}
		// elbow down position
		if self.up
		{
			theta2 *= -1.0;
		}

		// solve for theta 2
		let mut gamma = (y2 / x2).atan();
		let beta = (self.link2 * theta2.sin() / (self.link1 + self.link2 * theta2.cos())).atan();

		// ---------------------------------- theta 1 --------------------------------------
		// correct for quadrants
		if x2 < 0.0 && y2 > 0.0 
		{
			gamma += PI;
		} 
		else if x2 < 0.0 && y2 < 0.0 
		{
			gamma += PI;
		} 
		else if x2 > 0.0 && y2 < 0.0 
		{
			gamma += 2.0 * PI;
		}

		// find theta 1
		let mut theta1 = gamma - beta;

		// keep values between 1 and 2 PI
		if theta1 < 0.0 
		{
			theta1 += 2.0 * PI;
		}

		// ----------------------------- theta 3 ----------------------------------------
		let theta3 = si - theta2 - theta1;

		// ------------------------------ roll ------------------------------------------

		// update the struct

		Ok([theta1, theta2, theta3])
	}

}



// ----------------------------- tests -----------------------------------------------
#[cfg(test)]
mod tests 
{
	use super::*;
	use all_asserts::assert_near;


	#[test]
	fn test_joint_angles_nominal_theta1() 
	{
		// compare theta1 value to simulation value

		let arm = RoboticArm::new(10.0, 7.0, 3.0);

		let joint_angles = arm.find_joint_angles(-7.76, 6.9, 2.625).unwrap();
		let actual_angles: [f64; 2] = [1.55637, 2.29710];


		assert_near!(joint_angles[0], actual_angles[0], 0.001);
	}

	#[test]
	fn test_joint_angles_nominal_theta2() 
	{
		// compare theta2 value to simulation value
		let arm = RoboticArm::new(10.0, 7.0, 3.0);

		let joint_angles = arm.find_joint_angles(-7.76, 6.9, 2.625).unwrap();
		let actual_angles: [f64; 3] = [1.55637, 2.29710, 2.62535];


		assert_near!(joint_angles[1], actual_angles[1], 0.01);
	}

	#[test]
	fn test_singularity() 
	{
		// test if function produces singularity error when 
		// requested EF coordinates are out of workspace
		let arm = RoboticArm::new(10.0, 7.0, 3.0);
		let e = arm.find_joint_angles(21.0, -0.5, 0.0);

		match e {
			Ok(val) => {
				println!("Function should have errored, but returned: {:?}", val);
				panic!();
			},
			Err(e) => assert_eq!(e, 
				RoboticArmError::Singularity("Singularity it theta 2".into())),
		}
	}
}