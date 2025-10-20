use crate::constants::config;
use crate::constants::robotmap::drivetrain_map::{
    BL_DRIVE_ID, BL_ENCODER_ID, BL_TURN_ID, BR_DRIVE_ID, BR_ENCODER_ID, DRIVETRAIN_CANBUS,
    FL_DRIVE_ID, FL_ENCODER_ID, FL_TURN_ID, FR_DRIVE_ID, FR_ENCODER_ID, FR_TURN_ID, GYRO_ID,
};
use crate::swerve::kinematics::Kinematics;
use frcrs::ctre::{CanCoder, Pigeon, Talon};
use nalgebra::{Rotation2, Vector2, vector};
use uom::si::angle::{degree, revolution};
use uom::si::f64::Angle;

/// Drivetrain struct.
/// kinematics field interfaces with inverse kinematics functions.
/// motor_encoder_offsets are the absolute positions of the CANCoders on startup. These allow us to start the robot without physically zeroing the wheels.
pub struct Drivetrain {
    kinematics: Kinematics,
    gyro: Pigeon,

    motor_encoder_offsets: [Angle; 4],

    fl_encoder: CanCoder,
    fl_drive: Talon,
    fl_turn: Talon,

    bl_encoder: CanCoder,
    bl_drive: Talon,
    bl_turn: Talon,

    br_encoder: CanCoder,
    br_drive: Talon,
    br_turn: Talon,

    fr_encoder: CanCoder,
    fr_drive: Talon,
    fr_turn: Talon,
}

impl Drivetrain {
    /// Returns a new Drivetrain. CAN IDs and CanBus set in constants::robotmap::drivetrain_map
    pub fn new() -> Drivetrain {
        // make the encoders before rest of robot - we need them to get CANCoder offsets
        let fl_encoder = CanCoder::new(FL_ENCODER_ID, DRIVETRAIN_CANBUS);
        let bl_encoder = CanCoder::new(BL_ENCODER_ID, DRIVETRAIN_CANBUS);
        let br_encoder = CanCoder::new(BR_ENCODER_ID, DRIVETRAIN_CANBUS);
        let fr_encoder = CanCoder::new(FR_ENCODER_ID, DRIVETRAIN_CANBUS);

        // .get_absolute returns the CANCoder's rotation from -1 to 1
        let motor_encoder_offsets = [
            Angle::new::<revolution>(fl_encoder.get_absolute()),
            Angle::new::<revolution>(bl_encoder.get_absolute()),
            Angle::new::<revolution>(br_encoder.get_absolute()),
            Angle::new::<revolution>(fr_encoder.get_absolute()),
        ];

        Drivetrain {
            kinematics: Kinematics::new(),
            gyro: Pigeon::new(GYRO_ID, DRIVETRAIN_CANBUS),
            motor_encoder_offsets,

            fl_encoder,
            fl_drive: Talon::new(FL_DRIVE_ID, DRIVETRAIN_CANBUS),
            fl_turn: Talon::new(FL_TURN_ID, DRIVETRAIN_CANBUS),

            bl_encoder,
            bl_drive: Talon::new(BL_DRIVE_ID, DRIVETRAIN_CANBUS),
            bl_turn: Talon::new(BL_TURN_ID, DRIVETRAIN_CANBUS),

            br_encoder,
            br_drive: Talon::new(BR_DRIVE_ID, DRIVETRAIN_CANBUS),
            br_turn: Talon::new(BL_TURN_ID, DRIVETRAIN_CANBUS),

            fr_encoder,
            fr_drive: Talon::new(FR_DRIVE_ID, DRIVETRAIN_CANBUS),
            fr_turn: Talon::new(FR_TURN_ID, DRIVETRAIN_CANBUS),
        }
    }

    /// Stops the drivetrain.
    pub fn stop(&self) {
        self.fl_drive.stop();
        self.fl_turn.stop();

        self.bl_drive.stop();
        self.bl_turn.stop();

        self.br_drive.stop();
        self.br_turn.stop();

        self.fr_drive.stop();
        self.fr_turn.stop();
    }

    /// Resets the gyro.
    pub fn reset_heading(&mut self) {
        self.gyro.reset();
    }

    /// Field-orientate input from the driverstation.
    /// target_transformation is the x and y input from the driverstation put into a vector.
    /// This function rotates the driver's field orientated input to be robot oriented but the same direction.
    fn field_orientate(&self, target_transformation: Vector2<f64>) -> Vector2<f64> {
        Rotation2::new(-self.gyro.get_angle()) * target_transformation
    }

    /// Optimizes the setpoints.
    /// For example, instead of turning to 135 degrees from 0 degrees, turn to -45 degrees and invert speed.
    fn optimize_setpoints(&self, setpoints: Vec<(f64, Angle)>) -> Vec<(f64, Angle)> {
        // get angles, account for non-zero starting angle
        let mut measured_angles = vec![
            Angle::new::<revolution>(self.fl_turn.get_position()) + self.motor_encoder_offsets[0],
            Angle::new::<revolution>(self.bl_turn.get_position()) + self.motor_encoder_offsets[1],
            Angle::new::<revolution>(self.br_turn.get_position()) + self.motor_encoder_offsets[2],
            Angle::new::<revolution>(self.fr_turn.get_position()) + self.motor_encoder_offsets[3],
        ];

        measured_angles.iter().map(|angle| {
            let mut angle_value = angle.get::<degree>();
            angle_value = angle_value % 360.0;
            Angle::new::<degree>(angle_value)
        });

        setpoints.iter().map(|tuple| {
            let mut angle_value = tuple.1.get::<degree>();
            angle_value = angle_value % 360.0;
            (tuple.0, Angle::new::<degree>(angle_value))
        });

        for
    }

    /// Control the drivetrain.
    /// x, y, and rotation are driverstation inputs.
    pub fn control_drivetrain(&self, x: f64, y: f64, rotation: f64) {
        let target_transformation = match config::FIELD_ORIENTED {
            true => self.field_orientate(vector![x, y]),
            false => vector![x, y],
        };

        let mut targets = self.kinematics.get_targets(target_transformation, rotation);
        // TODO: IMPL TARGETS.OPTIMIZE (AHHHHHHHHHHHHHHHHHHHHHHHHHH)
        // TODO: IMPL SETSPEEDS
    }
}
