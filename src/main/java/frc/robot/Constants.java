// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class ModuleConstants {
        public static final double kPTurn = 1;
        public static final double kITurn = 0;
        public static final double kDTurn = 0.02;

        public static final double kDriveRotations2Meters = Constants.RobotMeasurements.WHEEL_CIRCUMFERENCE * Constants.RobotMeasurements.DRIVE_GEAR_RATIO;
        public static final double kDriveRPM2MetersPerSec = kDriveRotations2Meters / 60;
        public static final double kTurnRotations2Radians = Constants.RobotMeasurements.TURN_GEAR_RATIO * 2 * Math.PI;
        public static final double kTurnRPM2RadiansPerSec = kTurnRotations2Radians / 60;
    }

    public static final class DriveConstants {
        public static final double kDriveMaxSpeed = 10.0;
        public static final double kTurnMaxSpeed = 6 * Math.PI;

        public static final double kDeadband = 0.2;

        public static final double kMaxAccelerationPerSec = 3;
        public static final double kMaxTurnAccelerationPerSec = 3;

        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            new Translation2d(RobotMeasurements.kModuleToCenter, -RobotMeasurements.kModuleToCenter), 
            new Translation2d(-RobotMeasurements.kModuleToCenter, -RobotMeasurements.kModuleToCenter), 
            new Translation2d(RobotMeasurements.kModuleToCenter, RobotMeasurements.kModuleToCenter), 
            new Translation2d(-RobotMeasurements.kModuleToCenter, RobotMeasurements.kModuleToCenter)
            );
    }

    public static final class TeleConstants {
        public static final double kDriveMaxSpeed = DriveConstants.kDriveMaxSpeed / 2;
        public static final double kTurnMaxSpeed = DriveConstants.kTurnMaxSpeed / 2;
    }

    public static final class AutoConstants {
        public static final double kMaxSpeed = DriveConstants.kDriveMaxSpeed / 2;
        public static final double kMaxAccel = DriveConstants.kMaxAccelerationPerSec;
        public static final double kMaxTurnAccel = Math.PI / 4;
        public static final double kMaxTurnSpeed = DriveConstants.kTurnMaxSpeed / 2;

        public static final double kPX = 1.5;
        public static final double kIX = 0;
        public static final double kDX = 0;

        public static final double kPY = 1.5;
        public static final double kIY = 0;
        public static final double kDY = 0;

        public static final double kPTurn = 3;
        public static final double kITurn = 0;
        public static final double kDTurn = 0;
        public static final TrapezoidProfile.Constraints kAutoTurnControlConstraints = new TrapezoidProfile.Constraints(kMaxTurnSpeed, kMaxTurnAccel);
    }

    public static final class ControlConstants {
        public static final int DRIVER_JOYSTICK_PORT = 0;

        public static final int DRIVER_X_AXIS = 0;
        public static final int DRIVER_Y_AXIS = 1;
        public static final int DRIVER_TURN_AXIS = 4;

        public static final int DRIVER_FIELD_BUTTON_ID = 1;
        public static final int DRIVER_RESET_BUTTON_ID = 2;
    }

    public static final class RobotMap {
        public static final int FRONT_LEFT_DRIVE = 3;
        public static final int FRONT_LEFT_TURN = 4;

        public static final int FRONT_RIGHT_DRIVE = 6;
        public static final int FRONT_RIGHT_TURN = 5;

        public static final int BACK_LEFT_DRIVE = 8;
        public static final int BACK_LEFT_TURN = 7;

        public static final int BACK_RIGHT_DRIVE = 2;
        public static final int BACK_RIGHT_TURN = 1;

        public static final int FRONT_LEFT_ENCODER = 0;
        public static final int FRONT_RIGHT_ENCODER = 3;
        public static final int BACK_LEFT_ENCODER = 1;
        public static final int BACK_RIGHT_ENCODER = 2;

        public static final double FRONT_LEFT_MODULE_OFFSET = 0;
        public static final double FRONT_RIGHT_MODULE_OFFSET = 0;
        public static final double BACK_LEFT_MODULE_OFFSET = 0;
        public static final double BACK_RIGHT_MODULE_OFFSET = 0;
    }

    public static final class RobotMeasurements {
        public static final double WHEEL_RADIUS = Units.inchesToMeters(2);
        public static final double WHEEL_CIRCUMFERENCE = 2 * WHEEL_RADIUS * Math.PI;
        public static final double DRIVE_GEAR_RATIO = 6.86;
        public static final double TURN_GEAR_RATIO = 12.8;

        // public static final double FRONT_LEFT_MODULE_OFFSET = 0.10154637753865944;
        // public static final double FRONT_RIGHT_MODULE_OFFSET = 0.6477369411934235;
        // public static final double BACK_LEFT_MODULE_OFFSET = 0.7120562678014067;
        // public static final double BACK_RIGHT_MODULE_OFFSET = 0.5170884629272116;
        public static final double FRONT_LEFT_MODULE_OFFSET = 0;
        public static final double FRONT_RIGHT_MODULE_OFFSET = 0;
        public static final double BACK_LEFT_MODULE_OFFSET = 0;
        public static final double BACK_RIGHT_MODULE_OFFSET = 0;

        public static final double kModuleToModuleDistance = Units.inchesToMeters(22);
        public static final double kModuleToCenter = kModuleToModuleDistance / 2;
    }
}