use std::f64::consts::PI;
use nalgebra::{Rotation, Rotation2, Vector2};

/// ## Kinematics is a structure that stores vectors representing a swerve module's rotation unit vector.
/// The magnitude represents the speed of the module, and the direction of the vector represents the angle.
pub struct Kinematics {
    module_rotation_unit_vectors: Vec<Vector2<f64>>,
}

impl Kinematics {
    /// ## Calculates rotation unit vectors and returns a Kinematics.
    /// A rotation unit vector will rotate the robot on a dime when applied to the swerve modules.
    pub fn new() -> Kinematics {
        let half_width = crate::constants::config::WHEELBASE_WIDTH_INCHES / 2.0;
        let half_length = crate::constants::config::WHEELBASE_LENGTH_INCHES / 2.0;

        // vectors pointing to each module from center of robot.
        // convention is FL, BL, BR, FR
        let mut module_vectors: Vec<Vector2<f64>> = vec![
            Vector2::new(half_length, half_width), //   FL
            Vector2::new(-half_length, half_width), //  BL
            Vector2::new(-half_length, -half_width), // BR
            Vector2::new(half_length, -half_width), //  FR
        ];

        // rotate each vector by 90 degrees and normalize. This will give us the rotation unit vectors.
        let ninety_degree_rotation = Rotation2::new(f64::from(PI / 2.0));
        for mut vector in module_vectors.clone() {
            vector = ninety_degree_rotation * vector;
            vector = vector.normalize();
        }

        Kinematics { module_rotation_unit_vectors: module_vectors }
    }

    /// ## Given x, y, and rotation input from driver station, return a Vec<(f64, f64)> representing swerve module setpoints.
    /// Vec[1] = FL, 2 = BL, 3 = BR, 4 = FR.
    /// First f64 represents Speed, second f64 represents angle in radians.
    pub fn calculate_vectors(&self, x: f64, y: f64, input_rotation: f64) -> Vec<(f64, f64)> {
        let target_transformation = Vector2::new(x, y);
        let mut return_setpoints: Vec<(f64, f64)> = Vec::new();

        for mut rotation_unit_vector in &self.module_rotation_unit_vectors.clone() {
            // scale each rotation unit vector by the rotation amount.
            let rotation_vector = rotation_unit_vector.clone() * input_rotation;

            // add the scaled rotation vector to the target transformation vector in order to get the final vector.
            let mut final_vector = target_transformation + rotation_vector;
            final_vector = final_vector.normalize();

            // do some trig to figure out angle of final vector in radians
            let final_angle = f64::atan2(final_vector.y, final_vector.x);
            return_setpoints.push((final_vector.magnitude(), final_angle));
        }

        return_setpoints
    }
}