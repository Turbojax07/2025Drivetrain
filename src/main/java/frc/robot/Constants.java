package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;

public class Constants {
    public static final boolean isReplay = false;

    public class RobotMap {
        public static final int DT_FLDrive = 1; // CAN
        public static final int DT_FLSteer = 2; // CAN
        public static final int DT_FRDrive = 3; // CAN
        public static final int DT_FRSteer = 4; // CAN
        public static final int DT_BLDrive = 5; // CAN
        public static final int DT_BLSteer = 6; // CAN
        public static final int DT_BRDrive = 7; // CAN
        public static final int DT_BRSteer = 8; // CAN

        public static final int GYRO_Pigeon2Id = 9; // CAN

        public static final int DT_FLEncoder = 0; // Analog Pins
        public static final int DT_FREncoder = 1; // Analog Pins
        public static final int DT_BLEncoder = 2; // Analog Pins
        public static final int DT_BREncoder = 3; // Analog Pins
    }

    public class DriveConstants {
        public static final Distance robotWidth = Inches.of(24);
        public static final Distance robotLength = Inches.of(24);

        public static final LinearVelocity maxLinearVelocity = MetersPerSecond.of(6);
        public static final LinearAcceleration maxLinearAcceleration = MetersPerSecondPerSecond.of(3);
        public static final AngularVelocity maxAngularVelocity = RadiansPerSecond.of(3);
        public static final AngularAcceleration maxAngularAcceleration = RadiansPerSecondPerSecond.of(2);

        public static final Mass robotMass = Pounds.of(160);
        public static final MomentOfInertia robotMOI = KilogramSquareMeters.of(0.02);

        public static final Distance wheelRadius = Inches.of(4);

        public static final Current driveCurrentLimit = Amps.of(60);
        public static final Current steerCurrentLimit = Amps.of(30);

        public static final double kPDrive = 0.05 / wheelRadius.in(Meters);
        public static final double kIDrive = 0;
        public static final double kDDrive = 0;
        // TODO Find using SysID
        // public static final double kVDrive = 0;
        // public static final double kADrive = 0;

        public static final double kPSteer = 8;
        public static final double kISteer = 0;
        public static final double kDSteer = 0;

        public static final double driveGearRatio = 8.14;
        public static final double steerGearRatio = 150.0 / 7.0;

        public static final double driveMOI = 0.025;
        public static final double steerMOI = 0.004;

        public static final double metersPerRotation = 2 * Math.PI * wheelRadius.in(Meters) / driveGearRatio;

        public static final Translation2d flModuleOffset = new Translation2d(robotWidth.div( 2), robotLength.div( 2));
        public static final Translation2d frModuleOffset = new Translation2d(robotWidth.div( 2), robotLength.div(-2));
        public static final Translation2d blModuleOffset = new Translation2d(robotWidth.div(-2), robotLength.div( 2));
        public static final Translation2d brModuleOffset = new Translation2d(robotWidth.div(-2), robotLength.div(-2));

        public static final double flEncoderOffset = 0;
        public static final double frEncoderOffset = 0;
        public static final double blEncoderOffset = 0;
        public static final double brEncoderOffset = 0;

        public static final double[][] moduleConfigs = {
            {RobotMap.DT_FLDrive, RobotMap.DT_FLSteer, RobotMap.DT_FLEncoder, DriveConstants.flEncoderOffset}, // FL: drive id, steer id, encoder id, encoder offset
            {RobotMap.DT_FRDrive, RobotMap.DT_FRSteer, RobotMap.DT_FREncoder, DriveConstants.frEncoderOffset}, // FR: drive id, steer id, encoder id, encoder offset
            {RobotMap.DT_BLDrive, RobotMap.DT_BLSteer, RobotMap.DT_BLEncoder, DriveConstants.blEncoderOffset}, // BL: drive id, steer id, encoder id, encoder offset
            {RobotMap.DT_BRDrive, RobotMap.DT_BRSteer, RobotMap.DT_BREncoder, DriveConstants.brEncoderOffset}  // BR: drive id, steer id, encoder id, encoder offset
        };
    }
}