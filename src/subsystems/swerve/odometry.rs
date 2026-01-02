use crate::constants::drivetrain::{ARC_ODOMETRY_FOM_DAMPENING, ARC_ODOMETRY_MINIMUM_DELTA_ANGLE_RADIANS, SWERVE_DRIVE_RATIO, SWERVE_WHEEL_DIAMETER_INCHES};
use crate::subsystems::swerve::drivetrain::Drivetrain;
use nalgebra::{Rotation2, Vector2, vector};
use std::f64::consts::PI;
// use std::ops::Sub;
use uom::si::angle::{degree, radian, revolution};
use uom::si::f64::{Angle, Length};
use uom::si::length::{inch, meter};

/// ## Robot Odometry system.
/// last_frame_module_odometry: information about the swerve modules on the last frame update_odo was called. See the private struct ModuleOdometry for more.
pub struct Odometry {
    pub pose_estimate: RobotPoseEstimate,
    last_frame_module_odometry: Vec<ModuleOdometry>,
}

/// ## Private odometry struct that contains: <br>
/// -total linear distance the module's drive motor has traveled <br>
/// -module's current angle
// Clone lets us use .clone, Debug and PartialEq lets us use assert_eq!() in tests.
#[derive(Clone, Debug, PartialEq)]
struct ModuleOdometry {
    pub total_distance_traveled: Length,
    pub current_angle: Angle,
}

/// ## Where the robot thinks it is.
/// Note: We use Choreo's coordinate system (Rightmost blue driver station is (0,0), blue facing +x).
/// fom: figure of merit, how confident the robot is in its pose estimate.
#[derive(Clone)]
pub struct RobotPoseEstimate {
    fom: f64,
    x: Length,
    y: Length,
    angle: Angle,
}

impl Odometry {
    /// ## Makes a new Odometry system.
    /// The parameter RobotPoseEstimate will be where the robot starts from.
    pub fn new(pose: RobotPoseEstimate) -> Odometry {
        Odometry {
            pose_estimate: pose,
            last_frame_module_odometry: Vec::new(),
        }
    }
}

// impl Sub for ModuleOdometry {
//     type Output = ModuleOdometry;
//
//     fn sub(self, other: ModuleOdometry) -> ModuleOdometry {
//         ModuleOdometry {
//             total_distance_traveled: self.total_distance_traveled - other.total_distance_traveled,
//             current_angle: self.current_angle - other.current_angle,
//         }
//     }
// }

impl Drivetrain {
    /// ## Calculates module odometry.
    /// Note: ModuleOdometry is not an Angle and Speed. See ModuleOdometry struct for more.
    fn get_module_odometry(&self) -> Vec<ModuleOdometry> {
        let mut module_odometry = Vec::new();

        for (drive, turn) in [
            (&self.fl_drive, &self.fl_turn),
            (&self.bl_drive, &self.bl_turn),
            (&self.br_drive, &self.br_turn),
            (&self.fr_drive, &self.fr_turn),
        ] {
            module_odometry.push(ModuleOdometry {
                total_distance_traveled: Length::new::<inch>(
                    SWERVE_WHEEL_DIAMETER_INCHES * (drive.get_position() * SWERVE_DRIVE_RATIO),
                ),
                current_angle: Angle::new::<revolution>(turn.get_position()),
            })
        }

        module_odometry
    }

    pub(in crate::subsystems::swerve) fn set_next_frame_module_odometry(&mut self) {
        self.odometry.last_frame_module_odometry = self.get_module_odometry();
    }

    // TODO:
    // name things better
    // restructure for readability and efficiency
    //    use itertools::izip maybe
    /// ## Updates odometry for this frame.
    /// Uses Arc Odometry; see https://docs.google.com/document/d/1g-2a46vnE7GlO8Jhg7rIr4NdUOui1fEhV2Z8suaVDSE/edit?tab=t.0
    /// for a writeup by Riley LaMothe (2502) or team 1690's Software Sessions Part II. <br>
    /// Note: Does not fetch ModuleOdometry for this frame, intentionally.
    pub(in crate::subsystems::swerve) fn update_pose(&mut self) {
        let current_module_odometry = self.get_module_odometry();
        let last_frame_module_odometry = self.odometry.last_frame_module_odometry.clone();

        // Handle the first time this function is called; Odometry.last_frame_module_odometry is just a Vec::new().
        if last_frame_module_odometry.is_empty() {
            self.odometry.last_frame_module_odometry = current_module_odometry;
            return;
        }

        // Returns robot-oriented vectors representing each individual swerve module's pose change.
        let (robot_oriented_module_delta_poses, figure_of_merit) = module_level_arc_odometry(
            current_module_odometry.clone(),
            last_frame_module_odometry.clone(),
        );

        // Field-orient those vectors.
        let field_oriented_module_delta_pose =
            self.field_orient_delta_poses(robot_oriented_module_delta_poses);

        // Gets robot delta pose by averaging the module delta poses.
        let robot_delta_pose = average_module_delta_poses(field_oriented_module_delta_pose);

        let mut pose_estimate = self.odometry.pose_estimate.clone();
        pose_estimate.x += robot_delta_pose.x;
        pose_estimate.y += robot_delta_pose.y;
        pose_estimate.fom += figure_of_merit;
        pose_estimate.angle = Angle::new::<degree>(self.gyro.get_angle());

        self.odometry.pose_estimate = pose_estimate;
    }

    fn field_orient_delta_poses(
        &self,
        robot_oriented_delta_pose: Vec<Vector2<Length>>,
    ) -> Vec<Vector2<Length>> {
        let drivetrain_angle_rotation = Rotation2::new(self.gyro.get_angle() * PI / 180.0);

        let field_oriented_delta_pose: Vec<Vector2<Length>> = robot_oriented_delta_pose
            .iter()
            .map(|robot_oriented_delta_pose| {
                // Rotation2 only works on Vector2<f64>, not Vector2<Length>
                // Convert Vector2<Length> to Vector2<f64>
                let mut delta_position_f64_placeholder_meters = vector![
                    robot_oriented_delta_pose.x.get::<meter>(),
                    robot_oriented_delta_pose.x.get::<meter>()
                ];

                // Rotate (field orient) delta positon vector
                delta_position_f64_placeholder_meters =
                    drivetrain_angle_rotation * delta_position_f64_placeholder_meters;

                // Back to Vector2<Length>
                let field_oriented_delta_pose = vector![
                    Length::new::<meter>(delta_position_f64_placeholder_meters.x),
                    Length::new::<meter>(delta_position_f64_placeholder_meters.y)
                ];

                field_oriented_delta_pose
            })
            .collect();

        field_oriented_delta_pose
    }
}

/// ## Calculates each swerve module on the robot has moved since the last time this function was called, and a FOM.
/// Uses Arc Odometry; see https://docs.google.com/document/d/1g-2a46vnE7GlO8Jhg7rIr4NdUOui1fEhV2Z8suaVDSE/edit?tab=t.0 for a writeup by Riley LaMothe (2502) or team 1690's Software Sessions Part II.
fn module_level_arc_odometry(
    current_module_odometry: Vec<ModuleOdometry>,
    last_frame_module_odometry: Vec<ModuleOdometry>,
) -> (Vec<Vector2<Length>>, f64) {
    // Get change in module angle and distance traveled.
    // The distance traveled will be equal to the length of our imaginary arc.
    let (delta_angle, arc_length) =
        calculate_differences(&current_module_odometry, &last_frame_module_odometry);

    // Calculate arc's radius
    // We have the arc's angle (equal to change in module angle, via geometry) and the arc's length, so we can rewrite the following equation for radius
    // Arc Length = Arc Radius * Arc Angle in Radians   ->   Arc Length in Radians = Arc Length / Arc Radius
    let arc_radius: Vec<Length> = delta_angle
        .clone()
        .iter()
        .zip(arc_length.clone().iter())
        .map(|(delta_angle, arc_length)| {
            Length::new::<meter>(arc_length.get::<meter>() / delta_angle.get::<radian>())
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
        .zip(delta_angle.iter().zip(arc_radius.iter())) // Data structure is: Iterator<(Old ModuleOdometry, (delta_angle, arc_radius))>, that's what gets passed to the closure
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
    // Luckily, we can do the exact same thing we did to figure out the vector from the origin to the center.
    // To figure out the center to endpoint vector we use the current module angle in place of the old module angle and use the same code as above.
    // This will give us a vector that takes us from the endpoint to the center; if we subtract (or multiply the vector by -1 and add) this vector,
    //  we will have a vector that brings us from the arc center to the endpoint.
    let arc_center_to_endpoint_vector: Vec<Vector2<Length>> = current_module_odometry
        .clone()
        .iter()
        .zip(delta_angle.clone().iter().zip(arc_radius.iter())) // Data structure is: Iterator<(Current ModuleOdometry, (delta_angle, arc_radius))>, that's what gets passed to the closure
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
                -Length::new::<meter>(arc_radius.get::<meter>())
                    * endpoint_to_arc_center_angle.cos(),
                -Length::new::<meter>(arc_radius.get::<meter>())
                    * endpoint_to_arc_center_angle.sin(),
            ]
        })
        .collect();

    // Construct the final origin -> endpoint vector (finally).
    let _origin_to_endpoint_vector: Vec<Vector2<Length>> = origin_to_arc_center_vector
        .iter()
        .zip(arc_center_to_endpoint_vector.iter())
        .map(
            |(origin_to_arc_center_vector, arc_center_to_endpoint_vector)| {
                origin_to_arc_center_vector + arc_center_to_endpoint_vector
            },
        )
        .collect();

    // Figure out the delta position for all 4 modules.
    // If the delta_angle is too low the arc odometry will be very inaccurate. In this case, just assume a straight line.
    let delta_pose: Vec<Vector2<Length>> = origin_to_arc_center_vector
        .iter()
        .zip(delta_angle.iter())
        .zip(
            current_module_odometry
                .iter()
                .zip(last_frame_module_odometry.iter()),
        ) // Data structure: Iterator<((origin_to_arc_center_vector, delta_angle), (Current ModuleOdometry, Old ModuleOdometry))
        .map(
            |(
                (origin_to_arc_center_vector, delta_angle),
                (current_module_odometry, last_frame_module_odometry),
            )| {
                if delta_angle.get::<radian>().abs() < ARC_ODOMETRY_MINIMUM_DELTA_ANGLE_RADIANS
                    || delta_angle.get::<radian>().is_nan()
                {
                    vector![
                        current_module_odometry.total_distance_traveled
                            - last_frame_module_odometry.total_distance_traveled,
                        current_module_odometry.total_distance_traveled
                            - last_frame_module_odometry.total_distance_traveled
                    ]
                } else {
                    origin_to_arc_center_vector.to_owned()
                }
            },
        )
        .collect();

    // FOM calculation is DRIFT_RATIO * average_arc_length
    let mut sum_arc_length_meters_as_f64 = 0.0;
    for arc_length in arc_length.clone() {
        sum_arc_length_meters_as_f64 += arc_length.get::<meter>().abs();
    }
    let average_arc_length_meters_as_f64 = sum_arc_length_meters_as_f64 / arc_length.len() as f64;

    //

    // Look at graph of 1/(ax+1). A (ARC_ODOMETRY_FOM_DAMPENING) dampens higher values of x (avg. arc length), and the +1 makes sure y=1 for x=0.
    let figure_of_merit = 1.0 / ((ARC_ODOMETRY_FOM_DAMPENING) * average_arc_length_meters_as_f64 + 1.0);


    (delta_pose, figure_of_merit)
}

/// ## Calculates the changes in angle and distance between the current ModuleOdometry and the ModuleOdometry from last frame.
fn calculate_differences(
    current_module_odo: &Vec<ModuleOdometry>,
    last_frame_module_odo: &Vec<ModuleOdometry>,
) -> (Vec<Angle>, Vec<Length>) {
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
            current_module_odometry.total_distance_traveled
                - last_frame_module_odometry.total_distance_traveled
        })
        .collect();

    (delta_angle, delta_distance)
}

fn average_module_delta_poses(module_delta_poses: Vec<Vector2<Length>>) -> Vector2<Length> {
    // Cannot divide Vector<Length> by f64, but can divide Vector<f64> by f64
    let sum_delta_poses_length_vector = module_delta_poses.iter().sum::<Vector2<Length>>();
    let sum_delta_poses_f64_vector = vector![
        sum_delta_poses_length_vector.x.get::<meter>(),
        sum_delta_poses_length_vector.y.get::<meter>()
    ];

    let robot_delta_pose_f64_vector = sum_delta_poses_f64_vector / module_delta_poses.len() as f64;

    let robot_delta_pose_length_vector = vector![
        Length::new::<meter>(robot_delta_pose_f64_vector.x),
        Length::new::<meter>(robot_delta_pose_f64_vector.y)
    ];

    robot_delta_pose_length_vector
}

#[cfg(test)]
mod tests {
    use super::*;
    use uom::si::angle::degree;
    #[test]
    fn module_level_arc_odometry_stationary_test() {
        let current_module_odometry = vec![
            ModuleOdometry {
                total_distance_traveled: Length::new::<meter>(1.0),
                current_angle: Angle::new::<degree>(1.0),
            }, ModuleOdometry {
                total_distance_traveled: Length::new::<meter>(1.0),
                current_angle: Angle::new::<degree>(1.0),
            }, ModuleOdometry {
                total_distance_traveled: Length::new::<meter>(1.0),
                current_angle: Angle::new::<degree>(1.0),
            }, ModuleOdometry {
                total_distance_traveled: Length::new::<meter>(1.0),
                current_angle: Angle::new::<degree>(1.0),
            },
        ];

        let last_frame_module_odometry = vec![
            ModuleOdometry {
                total_distance_traveled: Length::new::<meter>(1.0),
                current_angle: Angle::new::<degree>(1.0),
            }, ModuleOdometry {
                total_distance_traveled: Length::new::<meter>(1.0),
                current_angle: Angle::new::<degree>(1.0),
            }, ModuleOdometry {
                total_distance_traveled: Length::new::<meter>(1.0),
                current_angle: Angle::new::<degree>(1.0),
            }, ModuleOdometry {
                total_distance_traveled: Length::new::<meter>(1.0),
                current_angle: Angle::new::<degree>(1.0),
            },
        ];

        let results = module_level_arc_odometry(current_module_odometry.clone(), last_frame_module_odometry.clone());

        let expected = (vec![
            vector![Length::new::<meter>(0.0), Length::new::<meter>(0.0)],
            vector![Length::new::<meter>(0.0), Length::new::<meter>(0.0)],
            vector![Length::new::<meter>(0.0), Length::new::<meter>(0.0)],
            vector![Length::new::<meter>(0.0), Length::new::<meter>(0.0)],
        ], 1.0);

        assert_eq!(results, expected);
    }

}
