use std::ops::Sub;
use nalgebra::{vector, Vector2};
use uom::si::length::{inch, meter};
use uom::si::angle::{degree, radian, revolution};
use uom::si::f64::{Angle, Length};
use crate::constants::drivetrain::{ARC_ODOMETRY_MINIMUM_DELTA_ANGLE_RADIANS, SWERVE_DRIVE_RATIO, SWERVE_WHEEL_DIAMETER_INCHES};
use crate::subsystems::swerve::drivetrain::Drivetrain;

/// ## Robot Odometry system.
/// last_frame_module_odometry: information about the swerve modules on the last frame update_odo was called. See the private struct ModuleOdometry for more.
pub struct Odometry {
    pub pose: RobotPoseEstimate,
    last_frame_module_odometry: Vec<ModuleOdometry>,
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

    // TODO:
    // abstract into multiple functions
    // robot -> field centric
    // individual modules -> robot pose
    // fom calc
    // add to current pos
    // name things better
    // restructure for readability and efficiency

    fn update_pose(&mut self) {
        let current_module_odometry = self.get_module_odometry();
        let last_frame_module_odometry = self.odometry.last_frame_module_odometry.clone();

        // Handle the first time this function is called; Odometry.last_frame_module_odometry is just a Vec::new().
        if last_frame_module_odometry.len() == 0 {
            self.odometry.last_frame_module_odometry = current_module_odometry;
            return;
        }

        // Arc Odometry starts here

        // Get change in module angle and distance traveled.
        // The distance traveled will be equal to the length of our imaginary arc.
        let (delta_angle, arc_length) = calculate_differences(&current_module_odometry, &last_frame_module_odometry);


        // Calculate arc's radius
        // We have the arc's angle (equal to change in module angle, via geometry) and the arc's length, so we can rewrite the following equation for radius
        // Arc Length = Arc Radius * Arc Angle in Radians   ->   Arc Length in Radians = Arc Length / Arc Radius
        let arc_radius: Vec<Length> = delta_angle.clone()
            .iter()
            .zip(arc_length.clone().iter())
            .map(|(delta_angle, arc_length)| {
                Length::new::<meter>(
                    arc_length.get::<meter>() / delta_angle.get::<radian>()
                )
            })
            .collect();


        // Calculate arc's center (represented by a mathematical vector), assuming last module is (0,0) w/ a robot-oriented coordinate system.
        // Currently, we know the arc's radius and the current and old module angles.
        // Via geometry (definition of tangency), the arc's center will be perpendicular to the old module's angle;
        //  the arc's center is perpendicular to where the module was facing in the past.
        // However, we don't know if the center is to the left of (0,0) or to the right; we can figure this out by seeing if the arc curves to the left or the right.
        // We can know if the arc curves to the left or the right via delta_angle.
        // After figuring out if it is to the left or right, we can simply go one radius that way to find the arc's center.

        // This is some scary syntax; just make sure you know what zip does, take your time, and you should be fine.
        let origin_to_arc_center_vector: Vec<Vector2<Length>> = last_frame_module_odometry
            .clone()
            .iter()
            .zip(
                delta_angle
                    .iter()
                    .zip(arc_radius.iter())
            )        // Data structure is: Iterator<(Old ModuleOdometry, (delta_angle, arc_radius))>, that's what gets passed to the closure
            .map(|(last_frame_module_odometry, (delta_angle, arc_radius))| {

                // Check if center is to left or right
                let mut origin_to_arc_center_angle = last_frame_module_odometry.current_angle;
                if delta_angle.get::<radian>() < 0.0 {
                    origin_to_arc_center_angle += Angle::new::<degree>(90.0);
                } else {
                    origin_to_arc_center_angle -= Angle::new::<degree>(90.0);
                }

                // Construct the vector with trig functions
                vector![
                    Length::new::<meter>(arc_radius.get::<meter>()) * origin_to_arc_center_angle.cos(),
                    Length::new::<meter>(arc_radius.get::<meter>()) * origin_to_arc_center_angle.sin(),
                ]
            })
            .collect();


        // Now, we have a vector that takes us from the origin to the center of the arc. If we get a vector that takes us from the center to the end point, we're good to go!
        // Luckily, we can do the exact same thing we did to figure out the vector from the origin to the center to figure out center to endpoint
        //  if we use the current module angle in place of the old module angle.
        // This will give us a vector that takes us from the endpoint to the center; if we subtract (or multiply the vector by -1 and add) this vector,
        //  we will have a vector that brings us from the arc center to the endpoint.
        let arc_center_to_endpoint_vector: Vec<Vector2<Length>> = current_module_odometry
            .clone()
            .iter()
            .zip(
                delta_angle
                    .clone()
                    .iter()
                    .zip(arc_radius.iter())
            )        // Data structure is: Iterator<(Current ModuleOdometry, (delta_angle, arc_radius))>, that's what gets passed to the closure
            .map(|(current_module_odometry, (delta_angle, arc_radius))| {

                // Check if center is to left or right
                let mut endpoint_to_arc_center_angle = current_module_odometry.current_angle;
                if delta_angle.get::<radian>() < 0.0 {
                    endpoint_to_arc_center_angle += Angle::new::<degree>(90.0);
                } else {
                    endpoint_to_arc_center_angle -= Angle::new::<degree>(90.0);
                }

                // Construct the vector with trig functions - Notice the negative signs in front, this changes the vector from
                // Endpoint -> Center to
                // Center -> Endpoint
                vector![
                    -Length::new::<meter>(arc_radius.get::<meter>()) * endpoint_to_arc_center_angle.cos(),
                    -Length::new::<meter>(arc_radius.get::<meter>()) * endpoint_to_arc_center_angle.sin(),
                ]
            })
            .collect();


        // Construct the final origin -> endpoint vector (finally).
        let origin_to_endpoint_vector: Vec<Vector2<Length>> = origin_to_arc_center_vector
            .iter()
            .zip(arc_center_to_endpoint_vector.iter())
            .map(|(origin_to_arc_center_vector, arc_center_to_endpoint_vector)| {
                origin_to_arc_center_vector + arc_center_to_endpoint_vector
            })
            .collect();


        // Figure out the delta position for all 4 modules.
        // If the delta_angle is too low the arc odometry will be very inaccurate. In this case, just assume a straight line.
        let delta_pose: Vec<Vector2<Length>> = origin_to_arc_center_vector
            .iter()
            .zip(delta_angle.iter())
            .zip(
                current_module_odometry
                    .iter()
                    .zip(last_frame_module_odometry.iter())
            ) // Data structure: Iterator<((origin_to_arc_center_vector, delta_angle), (Current ModuleOdometry, Old ModuleOdometry))
            .map(|((origin_to_arc_center_vector, delta_angle), (current_module_odometry, last_frame_module_odometry))| {
                if delta_angle.get::<radian>().abs() < ARC_ODOMETRY_MINIMUM_DELTA_ANGLE_RADIANS || delta_angle.get::<radian>().is_nan() {
                    vector![
                        current_module_odometry.total_distance_traveled - last_frame_module_odometry.total_distance_traveled,
                        current_module_odometry.total_distance_traveled - last_frame_module_odometry.total_distance_traveled
                    ]
                } else {
                    origin_to_arc_center_vector.to_owned()
                }
            })
            .collect();
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
    use uom::si::angle::{degree};
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