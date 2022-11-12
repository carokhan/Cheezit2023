package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class Constants {

    public static final class RobotMap {
        public static final int FRONT_LEFT_TURN = 4;
        public static final int FRONT_LEFT_DRIVE = 3;
        public static final int FRONT_LEFT_ENCODER = 0;
        public static final double FRONT_LEFT_OFFSET = 0;

        public static final int FRONT_RIGHT_TURN = 5;
        public static final int FRONT_RIGHT_DRIVE = 6;
        public static final int FRONT_RIGHT_ENCODER = 3;
        public static final double FRONT_RIGHT_OFFSET = 0;

        public static final int REAR_LEFT_TURN = 7;
        public static final int REAR_LEFT_DRIVE = 8;
        public static final int REAR_LEFT_ENCODER = 1;
        public static final double REAR_LEFT_OFFSET = 0; 

        public static final int REAR_RIGHT_TURN = 1;
        public static final int REAR_RIGHT_DRIVE = 2;
        public static final int REAR_RIGHT_ENCODER = 2;
        public static final double REAR_RIGHT_OFFSET = 0;
    }

    public static final class RobotMeasurements {
        public static final double WHEEL_RADIUS = Units.inchesToMeters(2);
        public static final double WHEEL_CIRCUMFERENCE = 2 * WHEEL_RADIUS * Math.PI;
        public static final double DRIVE_GEAR_RATIO = 6.86;
        public static final double TURN_GEAR_RATIO = 12.8;

        public static final double kModuleToModuleDistance = Units.inchesToMeters(22);
        public static final double kModuleToCenter = kModuleToModuleDistance / 2;
    }

    public static final class ControlConstants {
        public static final int kDriverPort = 0;
    }

    public static final class DriveConstants {
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            new Translation2d(RobotMeasurements.kModuleToCenter, -RobotMeasurements.kModuleToCenter), 
            new Translation2d(-RobotMeasurements.kModuleToCenter, -RobotMeasurements.kModuleToCenter), 
            new Translation2d(RobotMeasurements.kModuleToCenter, RobotMeasurements.kModuleToCenter), 
            new Translation2d(-RobotMeasurements.kModuleToCenter, RobotMeasurements.kModuleToCenter)
            );

        public static final double kMaxTurnSpeed = 1.0; // radians per second
        public static final double KMaxTurnAccel = 1.0; // radians per second^2

        public static final double kMaxSpeed = 3.75; // meters per second

        public static final double kDeadband = 0.05;

        public static final double kPRotation = 1;
        public static final double kIRotation = 0;
        public static final double kDRotation = 0.5;
    }

    public static final class AutoConstants {
        public static final double kMaxVelocity = 1.5; // meters per second
        public static final double kMaxAccel = 1; // meters per second^2

        public static final double kMaxAngularSpeed = 5; // radians per second
        public static final double kMaxAngularAccel = 5; // radians per second^2

        public static final Pose2d kTolerances = 
        new Pose2d(
            Units.inchesToMeters(0.5), 
            Units.inchesToMeters(0.5), 
            new Rotation2d(0)
        );

        public static final double kPTurn = 2;
        public static final double kITurn = 0;
        public static final double kDTurn = 0;

        public static final double kPDrive = 1.1;
        public static final double kIDrive = 0;
        public static final double KDDrive = 0.2;
    }
}
