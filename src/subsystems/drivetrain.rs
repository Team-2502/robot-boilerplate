use frcrs::ctre::{CanCoder, Pigeon, Talon};
use crate::swerve::kinematics::Kinematics;
use crate::constants::robotmap::drivetrain_map::{BL_DRIVE_ID, BL_TURN_ID, BR_DRIVE_ID, BR_ENCODER_ID, DRIVETRAIN_CANBUS, FL_DRIVE_ID, FL_ENCODER_ID, FL_TURN_ID, FR_DRIVE_ID, FR_ENCODER_ID, FR_TURN_ID, GYRO_ID};


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



        Drivetrain {
            kinematics: Kinematics::new(),
            gyro: Pigeon::new(GYRO_ID, DRIVETRAIN_CANBUS),

            fl_encoder: CanCoder::new(FL_ENCODER_ID, DRIVETRAIN_CANBUS),
            fl_drive: Talon::new(FL_DRIVE_ID, DRIVETRAIN_CANBUS),
            fl_turn: Talon::new(FL_TURN_ID, DRIVETRAIN_CANBUS),

            bl_encoder: CanCoder::new(FL_ENCODER_ID, DRIVETRAIN_CANBUS),
            bl_drive: Talon::new(BL_DRIVE_ID, DRIVETRAIN_CANBUS),
            bl_turn: Talon::new(BL_TURN_ID, DRIVETRAIN_CANBUS),

            br_encoder: CanCoder::new(BR_ENCODER_ID, DRIVETRAIN_CANBUS),
            br_drive: Talon::new(BR_DRIVE_ID, DRIVETRAIN_CANBUS),
            br_turn: Talon::new(BL_TURN_ID, DRIVETRAIN_CANBUS),

            fr_encoder: CanCoder::new(FR_ENCODER_ID, DRIVETRAIN_CANBUS),
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
}
