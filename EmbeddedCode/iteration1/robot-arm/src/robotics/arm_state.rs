/*
William Albertini


This module handles the movement of the robotic arm. It uses the 
inverse kinematics solver (InverseKinematicSolver) to update
joint positions and map them to encoder values. It also keeps
track of the initial state of the arm, and provides the change in
joint angles for any move. This is needed for the motor controllers,
which use relative position, not absolute position.

*/

use std::f64::consts::PI;
use std::ops::Sub;

use crate::networking::data_handler::DataHandler;
use crate::arm_errors::RoboticArmError;
use super::arm_kinematics::InverseKinematicSolver;



pub struct AngleToEncoderMap
{
    shoulder: u16,
    elbow: u16,
    wrist: u16,
    roll: u16,
    spool: u16,
}

impl AngleToEncoderMap
{
    pub fn new(shoulder: u16, elbow: u16, wrist: u16, roll: u16, spool: u16) -> AngleToEncoderMap
    {
        AngleToEncoderMap
        {
            shoulder,
            elbow,
            wrist,
            roll,
            spool
        }
    }
}

// struct to keep track of motor positions
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct ArmState
{
    pub shoulder: u16,
    pub elbow: u16,
    pub wrist: u16,
    pub roll: u16,
    pub spool: u16,
}

impl ArmState
{
    fn new(shoulder: u16, elbow: u16, wrist: u16, roll: u16, spool: u16) -> ArmState
    {
        ArmState{shoulder, elbow, wrist, roll, spool}
    }

    fn update(&mut self, shoulder: u16, elbow: u16, wrist: u16, roll: u16, spool: u16) 
    {
        self.shoulder = shoulder;
        self.elbow = elbow;
        self.wrist = wrist;
        self.roll = roll;
        self.spool = spool;

    }

    fn update_kinematic_joints(&mut self, shoulder: u16, elbow: u16, wrist: u16)
    {
        // these joints are found by the solver
        self.shoulder = shoulder;
        self.elbow = elbow;
        self.wrist = wrist;
    }
}

impl Sub for ArmState
{
    type Output = Self;

    // allows for the comparison of the motor states
    fn sub(self, state: ArmState) -> Self::Output
    {

        ArmState::new(
            self.shoulder - state.shoulder,
            self.elbow - state.elbow,
            self.wrist - state.wrist,
            self.roll - state.roll,
            self.spool - state.spool,
        )
    }
}

pub struct RoboticArmSolver
{
    x: f64,
    y: f64,
    si: f64,
    initial_state: ArmState,
    updated_state: ArmState,
    solver: InverseKinematicSolver,
    joint_map: AngleToEncoderMap,
}


impl RoboticArmSolver {

    pub fn try_new_from_ef_position(link1: f64, 
                                    link2: f64, 
                                    link3: f64, 
                                    starting_x: f64, 
                                    starting_y: f64, 
                                    starting_si: f64, 
                                    joint_map: AngleToEncoderMap) -> Result<RoboticArmSolver, RoboticArmError> 
    {
        // create new end effector position
        let solver = InverseKinematicSolver::new(link1, link2, link3);

        // find initial joint values
        let [theta1, theta2, theta3] = solver.find_joint_angles(starting_x, starting_y, starting_si)?;

        // convert the joint angles (radians) to encoder ticks
        let init_shoulder = (theta1 * joint_map.shoulder as f64 / 2.0 / PI) as u16;
        let init_elbow = (theta2 * joint_map.elbow as f64 / 2.0 / PI) as u16;
        let init_wrist = (theta3 * joint_map.wrist as f64 / 2.0 / PI) as u16;

        // create init and updated ArmState
        let initial_state = ArmState::new(init_shoulder, init_elbow, init_wrist, 0, 0);
        let updated_state = initial_state.clone();

        Ok(RoboticArmSolver{ 
            x: starting_x,
            y: starting_y,
            si: starting_si,
            initial_state,
            updated_state,
            solver,
            joint_map,
        })
    }

    pub fn update_from_data_handler(&mut self, data: DataHandler) -> Result<(), RoboticArmError>
    {
        // close fingers depending on buttons
        if data.both_buttons_pressed()
        {
            // **** IMPLEMENT A GO HOME ROUTINE

        // if only button 2 is pressed, increment spool
        } else if (data.button1 == 1) && (data.button2 == 0)
        {
            self.updated_state.spool = self.add_value_wrap(self.updated_state.spool, 8, self.joint_map.spool);
        // if only button 1 is pressed, decrement spool
        } else if (data.button1 == 0) && (data.button2 == 1)
        {
            self.updated_state.spool = self.add_value_wrap(self.updated_state.spool, -8, self.joint_map.spool);
        }

        // update roll
        self.updated_state.roll = self.add_value_wrap(self.updated_state.roll, data.roll as i32, self.joint_map.roll);

        // try to update state of arm
        let new_x = data.x as f64 + self.x;
        let new_y = data.y as f64 + self.y;
        let new_pitch = data.pitch as f64 / 100.0 + self.si;

        let kinematics_result = self.solver.find_joint_angles(new_x, new_y, new_pitch);
        
        match kinematics_result
        {
            Ok(joint_angles) => {
                // update new end effector positions
                self.x = new_x;
                self.y = new_y;
                self.si = new_pitch;

                // convert motor positions to u16 motor values (these will be sent directly to motor controllers)
                let shoulder_position = (joint_angles[0] * self.joint_map.shoulder as f64/ 2.0 / PI) as u16;
                let elbow_position = (joint_angles[1] * self.joint_map.elbow as f64 / 2.0 / PI) as u16;
                let wrist_position = (joint_angles[2] * self.joint_map.wrist as f64 / 2.0 / PI) as u16;
                
                // update updated state
                self.updated_state.update_kinematic_joints(shoulder_position, elbow_position, wrist_position);

                Ok(())

            }
            Err(e) => return Err(RoboticArmError::KinematicJointsNotUpdated("Joints not updated due to singularity".into())),
        }
    }

    pub fn get_delta_joints(&self) -> ArmState
    {
        // return the overall change in the joints
        self.updated_state - self.initial_state
    }

    fn add_value_wrap(&self, curr_value: u16, add: i32, map_value: u16) -> u16
    {
        // this function keeps the motor position between 0-max_encoder_value
        if add >= 0
        {
            
            let updated_value = curr_value + add as u16;
            if updated_value >= map_value
            {
                return updated_value - map_value
            } else {
                return updated_value
            }
        } else {
            let updated_value = (curr_value as i32) + add;

            if updated_value < 0
            {
                return (map_value as i32 + updated_value) as u16
            } else {
                return updated_value as u16
            }
        }
    }
}




// ------------------------------- unit tests -------------------------------------
#[cfg(test)]
mod tests
{
    use super::*;

    #[test]
    fn test_starting_straight()
    {
        // initilize arm at very limit of workspace
        let map = AngleToEncoderMap::new(5000, 5000, 5000, 5000, 5000);
        let default_state = ArmState{ shoulder: 0, elbow: 0, wrist: 0, roll: 0, spool:0 };
        let robotic_arm = RoboticArmSolver::try_new_from_ef_position(10.0, 
                                                                                    5.0, 
                                                                                    3.0, 
                                                                                    18.0, 0.0, 
                                                                                    0.0, map).expect("Contructor did not work");
        assert_eq!(robotic_arm.initial_state, default_state);
    }

    #[test]
    fn test_starting_out_of_workspace()
    {
        // initialize end effector out of the workspace
        let map = AngleToEncoderMap::new(5000, 5000, 5000, 5000, 5000);
        let robotic_arm = RoboticArmSolver::try_new_from_ef_position(10.0, 
                                                                                    5.0, 
                                                                                    3.0, 
                                                                                    19.0, 0.0, 
                                                                                    0.0, map);
        if let Err(e) = robotic_arm
        {
            assert_eq!(e, RoboticArmError::Singularity("Singularity in theta 2".into()));
        } else {
            assert!(false, "Function call should have errored out");
        }
    }

    #[test]
    fn test_moving_ef_out_of_workspace()
    {
        // test moving end effector out of workspace (no update should occur)
        let data = DataHandler::new(8, 0, 0, 0, 1, 1);
        let map = AngleToEncoderMap::new(5000, 5000, 5000, 5000, 5000);
        let mut robotic_arm = RoboticArmSolver::try_new_from_ef_position(10.0, 
                                                                    5.0, 
                                                                    3.0, 
                                                                    18.0, 0.0, 
                                                                    0.0, map).unwrap();
        let update_status = robotic_arm.update_from_data_handler(data);
        
        // data should make EF move out of workspace
        if let Err(e) = update_status
        {
            assert_eq!(e, RoboticArmError::KinematicJointsNotUpdated("Joints not updated due to singularity".into()));
        } else {
            assert!(false, "EF was able to move out of workspace")
        }

        let joint_state = robotic_arm.get_delta_joints();

        // check to make sure the change in joints (from initial to move) is 0
        assert_eq!(joint_state, ArmState::new(0,0,0,0,0));
    }

    #[test]
    fn test_double_roll()
    {
        // test rolling/spooling the arm twice in succession
        let data = DataHandler::new(8, 0, 8, 0, 1, 0);
        let map = AngleToEncoderMap::new(5000, 5000, 5000, 5000, 5000);
        let mut robotic_arm = RoboticArmSolver::try_new_from_ef_position(10.0, 
                                                                    5.0, 
                                                                    3.0, 
                                                                    18.0, 0.0, 
                                                                    0.0, map).unwrap();

        let mut update_status: Result<(), RoboticArmError> = robotic_arm.update_from_data_handler(data.clone());
        update_status = robotic_arm.update_from_data_handler(data);
        
        // data should make EF move out of workspace
        if let Err(e) = update_status
        {
            assert_eq!(e, RoboticArmError::KinematicJointsNotUpdated("Joints not updated due to singularity".into()));
        } else {
            assert!(false, "EF was able to move out of workspace")
        }

        let joint_state = robotic_arm.get_delta_joints();

        // check to make sure the change in joints (from initial to move) is 0
        assert_eq!(joint_state, ArmState::new(0,0,0,16,16));
    }

    #[test]
    fn test_roll_spool_update_with_bad_ef_update()
    {
        // This test aims to update spool and roll values despite a faulty EF position update
        // The robot should still update spool and roll while returning error for invalid EF
        // position
        let data = DataHandler::new(8, 0, 8, 0, 1, 0);
        let map = AngleToEncoderMap::new(5000, 5000, 5000, 5000, 5000);
        let mut robotic_arm = RoboticArmSolver::try_new_from_ef_position(10.0, 
                                                                    5.0, 
                                                                    3.0, 
                                                                    18.0, 0.0, 
                                                                    0.0, map).unwrap();

        let update_status = robotic_arm.update_from_data_handler(data);

        // data should make EF move out of workspace
        if let Err(e) = update_status
        {
            assert_eq!(e, RoboticArmError::KinematicJointsNotUpdated("Joints not updated due to singularity".into()));
        } else {
            assert!(false, "EF was able to move out of workspace")
        }

        // Arm state should be updated
        let delta_joints = robotic_arm.get_delta_joints();

        assert_eq!(delta_joints, ArmState::new(0, 0, 0, 8, 8));

    }

    #[test]
    fn test_overflow_wrap()
    {
        // test overflow functionality 
        let map = AngleToEncoderMap::new(5000, 5000, 5000, 5000, 5000);
        let robotic_arm = RoboticArmSolver::try_new_from_ef_position(10.0, 
            5.0, 
            3.0, 
            18.0, 0.0, 
            0.0, map).expect("Contructor did not work");

        assert_eq!(robotic_arm.add_value_wrap(4999, 7, 5000), 6);
    }

    #[test]
    fn test_underflow_wrap()
    {
        // test undeflow functionality
        let map = AngleToEncoderMap::new(5000, 5000, 5000, 5000, 5000);
        let robotic_arm = RoboticArmSolver::try_new_from_ef_position(10.0, 
            5.0, 
            3.0, 
            18.0, 0.0, 
            0.0, map).expect("Contructor did not work");

        assert_eq!(robotic_arm.add_value_wrap(2, -3, 5000), 4999);
    }

    #[test]
    fn test_edge_overflow_wrap()
    {
        // test the exact value that overflow occurs at
        let map = AngleToEncoderMap::new(5000, 5000, 5000, 5000, 5000);
        let robotic_arm = RoboticArmSolver::try_new_from_ef_position(10.0, 
            5.0, 
            3.0, 
            18.0, 0.0, 
            0.0, map).expect("Contructor did not work");

        assert_eq!(robotic_arm.add_value_wrap(4998, 2, 5000), 0);
    }

    #[test]
    fn test_edge_undeflow_wrap()
    {
        // test the underflow function at 0
        let map = AngleToEncoderMap::new(5000, 5000, 5000, 5000, 5000);
        let robotic_arm = RoboticArmSolver::try_new_from_ef_position(10.0, 
            5.0, 
            3.0, 
            18.0, 0.0, 
            0.0, map).expect("Contructor did not work");

        assert_eq!(robotic_arm.add_value_wrap(2, -2, 5000), 0);
    }
}