use uom::si::length::inch;
use uom::si::angle::revolution;
use uom::si::f64::{Angle, Length};
use crate::constants::drivetrain::{SWERVE_DRIVE_RATIO, SWERVE_WHEEL_DIAMETER_INCHES};
use crate::subsystems::swerve::drivetrain::Drivetrain;

/// ## Robot Odometry system.
/// last_frame_module_odometry: information about the swerve modules on the last frame update_odo was called. See the private struct ModuleOdometry for more.
pub struct Odometry {
    pub pose: RobotPoseEstimate,
    last_frame_module_odometry: Vec<ModuleOdometry>,
}

impl Odometry {
    /// ## Makes a new Odometry system.
    /// The parameter RobotPoseEstimate will be where the robot starts from.
    pub fn new(pose: RobotPoseEstimate) -> Odometry {
        Odometry {
            pose,
            last_frame_module_odometry: Vec::new(),
        }
    }

    pub fn set_pose(&mut self, pose: RobotPoseEstimate) {
        self.pose = pose;
    }
}

/// ## Private odometry struct that contains: <br>
/// -total linear distance the module's drive motor has traveled <br>
/// -module's current angle
#[derive(Clone)]
struct ModuleOdometry {
    pub total_distance_traveled: Length,
    pub current_angle: Angle,
}

/// ## Where the robot thinks it is.
/// Note: We use Choreo's coordinate system (Rightmost blue driver station is (0,0), blue facing +x).
/// fom: figure of merit, how confident the robot is in its pose estimate.
pub struct RobotPoseEstimate {
    fom: f64,
    x: Length,
    y: Length,
}

impl Drivetrain {
    /// ## Calculates module odometry.
    /// Note: ModuleOdometry is not an Angle and Speed. See ModuleOdometry struct for more.
    fn get_module_odometry(&self) -> Vec<ModuleOdometry> {
        let mut module_odometry = Vec::new();

        for (drive, turn) in [
            (&self.fl_drive, &self.fl_turn),
            (&self.bl_drive, &self.bl_turn),
            (&self.br_drive, &self.br_turn),
            (&self.fr_drive, &self.fr_turn)
        ] {
            module_odometry.push(
                ModuleOdometry {
                    total_distance_traveled: Length::new::<inch>(SWERVE_WHEEL_DIAMETER_INCHES * (drive.get_position() * SWERVE_DRIVE_RATIO)),
                    current_angle: Angle::new::<revolution>(turn.get_position()),
                }
            )
        }

        module_odometry
    }

    /// ## Calculates how the robot's x and y has moved since the last time this function was called.
    /// Uses Arc Odometry; see https://docs.google.com/document/d/1g-2a46vnE7GlO8Jhg7rIr4NdUOui1fEhV2Z8suaVDSE/edit?tab=t.0 for a writeup by Riley LaMothe (2502) or team 1690's Software Sessions Part II.
    fn update_pose(&mut self) {
        let current_module_odo = self.get_module_odometry();
        let last_frame_module_odo = self.odometry.last_frame_module_odometry.clone();

        // Handle the first time this function is called; Odometry.last_frame_module_odometry is just a Vec::new().
        if last_frame_module_odo.len() == 0 {
            self.odometry.last_frame_module_odometry = current_module_odo;
            return;
        }

        // Get change in angle and distance traveled
        let (delta_angle, distance) = calculate_differences(&current_module_odo, &last_frame_module_odo);


    }

    fn update_odo(&mut self) {


    }
}

/// ## Calculates the changes in angle and distance between the current ModuleOdometry and the ModuleOdometry from last frame.
fn calculate_differences(current_module_odo: &Vec<ModuleOdometry>, last_frame_module_odo: &Vec<ModuleOdometry>) -> (Vec<Angle>, Vec<Length>) {
    let delta_angle: Vec<Angle> = current_module_odo
        .iter()
        .zip(last_frame_module_odo.clone()) // Data structure is now: Iterator<(Current ModuleOdometry, Old ModuleOdometry)>
        .map(|(current_module_odometry, last_frame_module_odometry)| {
            current_module_odometry.current_angle - last_frame_module_odometry.current_angle
        })
        .collect();

    let delta_distance: Vec<Length> = current_module_odo
        .iter()
        .zip(last_frame_module_odo) // Data structure is now: Iterator<(Current ModuleOdometry, Old ModuleOdometry)>
        .map(|(current_module_odometry, last_frame_module_odometry)| {
            current_module_odometry.total_distance_traveled - last_frame_module_odometry.total_distance_traveled
        })
        .collect();

    (delta_angle, delta_distance)
}

#[cfg(test)]
mod tests {
    use uom::si::angle::{degree, radian};
    use super::*;
    #[test]
    fn delta_angle() {
        let len0 = Length::new::<inch>(0.0);
        let current_module_odo = [
            ModuleOdometry {total_distance_traveled: len0, current_angle: Angle::new::<degree>(90.0)},
            ModuleOdometry {total_distance_traveled: len0, current_angle: Angle::new::<degree>(0.0)},
            ModuleOdometry {total_distance_traveled: len0, current_angle: Angle::new::<degree>(180.0)},
            ModuleOdometry {total_distance_traveled: len0, current_angle: Angle::new::<degree>(0.0)},
        ];
        let last_frame_module_odo = [
            ModuleOdometry {total_distance_traveled: len0, current_angle: Angle::new::<degree>(0.0)},
            ModuleOdometry {total_distance_traveled: len0, current_angle: Angle::new::<degree>(90.0)},
            ModuleOdometry {total_distance_traveled: len0, current_angle: Angle::new::<degree>(0.0)},
            ModuleOdometry {total_distance_traveled: len0, current_angle: Angle::new::<degree>(180.0)},
        ];

        let delta_angle: Vec<Angle> = current_module_odo
            .iter()
            .zip(last_frame_module_odo) // Data structure is now: Iterator<(Current ModuleOdometry, Old ModuleOdometry)>
            .map(|(current_module_odometry, last_frame_module_odometry)| {
                current_module_odometry.current_angle - last_frame_module_odometry.current_angle
            })
            .collect();


        println!("{:?}, {:?}, {:?}, {:?},", delta_angle[0].get::<degree>(), delta_angle[1].get::<degree>(), delta_angle[2].get::<degree>(), delta_angle[3].get::<degree>());
        assert_eq!(delta_angle, [Angle::new::<degree>(90.0), Angle::new::<degree>(-90.0), Angle::new::<degree>(180.0), Angle::new::<degree>(-180.0)]);
    }
}