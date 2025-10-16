use std::f64::consts::PI;
use nalgebra::{Rotation, Rotation2, Vector2};

/// ## Kinematics is a structure that stores vectors representing a swerve module's rotation unit vector.
/// The magnitude represents the speed of the module, and the direction of the vector represents the angle.
/// While the rotation vectors are constant, we don't want to calculate them each frame, and syntax like drivetrain.kinematics.calculate_targets is intuitive.
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
        let mut final_vectors:Vec<Vector2<f64>> = Vec::new();
        let ninety_degree_rotation = Rotation2::new(f64::from(PI / 2.0));
        for mut vector in module_vectors {
            vector = ninety_degree_rotation * vector;
            vector = vector.normalize();
            final_vectors.push(vector);
        }

        Kinematics { module_rotation_unit_vectors: final_vectors }
    }

    /// ## Given x, y, and rotation input from driver station, return a Vec<(f64, f64)> representing swerve module setpoints.
    /// Vec(1) = FL, 2 = BL, 3 = BR, 4 = FR.
    /// First f64 represents speed, second f64 represents angle in RADIANS, wrapped from -PI to PI.
    pub fn calculate_targets(&self, x: f64, y: f64, input_rotation: f64) -> Vec<(f64, f64)> {
        let target_transformation = Vector2::new(x, y);
        let mut module_setpoints: Vec<(f64, f64)> = Vec::new();

        for mut rotation_unit_vector in &self.module_rotation_unit_vectors.clone() {
            // scale each rotation unit vector by the rotation amount.
            let rotation_vector = rotation_unit_vector.clone() * input_rotation;

            // add the scaled rotation vector to the target transformation vector in order to get the final vector.
            let mut final_vector = target_transformation + rotation_vector;
            final_vector = final_vector;

            // do some trig to figure out angle of final vector in radians
            let final_angle = f64::atan2(final_vector.y, final_vector.x);
            module_setpoints.push((final_vector.magnitude(), final_angle));
        }

        module_setpoints
    }
}


// run tests with
//      cargo test -- --nocapture
// to show prints even for successful tests.
// by default, rust will capture (delete) output (for our context, println!) from successful tests. --nocapture prevents that.
#[cfg(test)]
mod kinematics_tests {
    use nalgebra::vector;
    use super::*;

    #[test]
    fn kinematics_new_test() {
        let results = Kinematics::new();

        // these seemingly random numbers are coordinates on the unit circle, specifically sqrt2/2 = 0.707..
        let expected = vec![
            vector![-0.7071067811865475, 0.7071067811865475],
            vector![-0.7071067811865475, -0.7071067811865475],
            vector![0.7071067811865475, -0.7071067811865475],
            vector![0.7071067811865475, 0.7071067811865475],
        ];

        println!("expected: {:?}", expected);
        println!("results: {:?}", results.module_rotation_unit_vectors);
        assert_eq!(results.module_rotation_unit_vectors, expected);
    }

    #[test]
    fn calculate_targets_right_full_power_test() {
        println!("calculate_targets_right_test:");
        let kinematics = Kinematics::new();

        let results = kinematics.calculate_targets(1.0, 0.0, 0.0);
        let expected: Vec<(f64, f64)> = vec![
            (1.0, 0.0),
            (1.0, 0.0),
            (1.0, 0.0),
            (1.0, 0.0),
        ];
        println!("expected: {:?}", expected);
        println!("results: {:?}", results);
        assert_eq!(expected, results);
    }

    #[test]
    fn calculate_targets_right_half_power_test() {
        println!("calculate_targets_right_test:");
        let kinematics = Kinematics::new();

        let results = kinematics.calculate_targets(0.5, 0.0, 0.0);
        let expected: Vec<(f64, f64)> = vec![
            (0.5, 0.0),
            (0.5, 0.0),
            (0.5, 0.0),
            (0.5, 0.0),
        ];
        println!("expected: {:?}", expected);
        println!("results: {:?}", results);
        assert_eq!(expected, results);
    }

    #[test]
    fn calculate_targets_left_full_power_test() {
        println!("calculate_targets_left_test:");
        let kinematics = Kinematics::new();

        let results = kinematics.calculate_targets(-1.0, 0.0, 0.0);
        let expected: Vec<(f64, f64)> = vec![
            (1.0, PI),
            (1.0, PI),
            (1.0, PI),
            (1.0, PI),
        ];
        println!("expected: {:?}", expected);
        println!("results: {:?}", results);
        assert_eq!(expected, results);
    }

    #[test]
    fn calculate_targets_left_half_power_test() {
        println!("calculate_targets_left_test:");
        let kinematics = Kinematics::new();

        let results = kinematics.calculate_targets(-0.5, 0.0, 0.0);
        let expected: Vec<(f64, f64)> = vec![
            (0.5, PI),
            (0.5, PI),
            (0.5, PI),
            (0.5, PI),
        ];
        println!("expected: {:?}", expected);
        println!("results: {:?}", results);
        assert_eq!(expected, results);
    }

    #[test]
    fn calculate_targets_up_full_power_test() {
        println!("calculate_targets_up_test:");
        let kinematics = Kinematics::new();

        let results = kinematics.calculate_targets(0.0, 1.0, 0.0);
        let expected: Vec<(f64, f64)> = vec![
            (1.0, PI/2.0),
            (1.0, PI/2.0),
            (1.0, PI/2.0),
            (1.0, PI/2.0),
        ];
        println!("expected: {:?}", expected);
        println!("results: {:?}", results);
        assert_eq!(expected, results);
    }

    #[test]
    fn calculate_targets_up_half_power_test() {
        println!("calculate_targets_up_test:");
        let kinematics = Kinematics::new();

        let results = kinematics.calculate_targets(0.0, 0.5, 0.0);
        let expected: Vec<(f64, f64)> = vec![
            (0.5, PI/2.0),
            (0.5, PI/2.0),
            (0.5, PI/2.0),
            (0.5, PI/2.0),
        ];
        println!("expected: {:?}", expected);
        println!("results: {:?}", results);
        assert_eq!(expected, results);
    }

    #[test]
    fn calculate_targets_down_full_power_test() {
        println!("calculate_targets_down_test:");
        let kinematics = Kinematics::new();

        let results = kinematics.calculate_targets(0.0, -1.0, 0.0);
        let expected: Vec<(f64, f64)> = vec![
            (1.0, PI/-2.0),
            (1.0, PI/-2.0),
            (1.0, PI/-2.0),
            (1.0, PI/-2.0),
        ];
        println!("expected: {:?}", expected);
        println!("results: {:?}", results);
        assert_eq!(expected, results);
    }

    #[test]
    fn calculate_targets_down_half_power_test() {
        println!("calculate_targets_down_test:");
        let kinematics = Kinematics::new();

        let results = kinematics.calculate_targets(0.0, -0.5, 0.0);
        let expected: Vec<(f64, f64)> = vec![
            (0.5, PI/-2.0),
            (0.5, PI/-2.0),
            (0.5, PI/-2.0),
            (0.5, PI/-2.0),
        ];
        println!("expected: {:?}", expected);
        println!("results: {:?}", results);
        assert_eq!(expected, results);
    }

    #[test]
    fn calculate_targets_clockwise_full_power_test() {
        println!("calculate_targets_clockwise_test:");
        let kinematics = Kinematics::new();

        let results = kinematics.calculate_targets(0.0, 0.0, 1.0);
        // floating point operations means 1.0 becomes 0.9999999999
        let expected: Vec<(f64, f64)> = vec![
            (0.9999999999999999, (3.0*PI)/4.0),
            (0.9999999999999999, (-3.0*PI)/4.0),
            (0.9999999999999999, -PI/4.0),
            (0.9999999999999999, PI/4.0),
        ];
        println!("expected: {:?}", expected);
        println!("results: {:?}", results);
        assert_eq!(expected, results);
    }

    #[test]
    fn calculate_targets_clockwise_half_power_test() {
        println!("calculate_targets_clockwise_test:");
        let kinematics = Kinematics::new();

        let results = kinematics.calculate_targets(0.0, 0.0, 0.5);
        // floating point operations means 0.5 becomes 0.49999999999999994
        let expected: Vec<(f64, f64)> = vec![
            (0.49999999999999994, (3.0*PI)/4.0),
            (0.49999999999999994, (-3.0*PI)/4.0),
            (0.49999999999999994, -PI/4.0),
            (0.49999999999999994, PI/4.0),
        ];
        println!("expected: {:?}", expected);
        println!("results: {:?}", results);
        assert_eq!(expected, results);
    }

    #[test]
    fn calculate_targets_counter_clockwise_full_power_test() {
        println!("calculate_targets_clockwise_test:");
        let kinematics = Kinematics::new();

        let results = kinematics.calculate_targets(0.0, 0.0, -1.0);
        // floating point operations means 1.0 becomes 0.9999999999
        let expected: Vec<(f64, f64)> = vec![
            (0.9999999999999999, -PI/4.0),
            (0.9999999999999999, PI/4.0),
            (0.9999999999999999, (3.0*PI)/4.0),
            (0.9999999999999999, (-3.0*PI)/4.0),
        ];
        println!("expected: {:?}", expected);
        println!("results: {:?}", results);
        assert_eq!(expected, results);
    }

    #[test]
    fn calculate_targets_counter_clockwise_half_power_test() {
        println!("calculate_targets_clockwise_test:");
        let kinematics = Kinematics::new();

        let results = kinematics.calculate_targets(0.0, 0.0, -0.5);
        // floating point operations means 0.5 becomes 0.49999999999999994
        let expected: Vec<(f64, f64)> = vec![
            (0.49999999999999994, -PI/4.0),
            (0.49999999999999994, PI/4.0),
            (0.49999999999999994, (3.0*PI)/4.0),
            (0.49999999999999994, (-3.0*PI)/4.0),
        ];
        println!("expected: {:?}", expected);
        println!("results: {:?}", results);
        assert_eq!(expected, results);
    }

}