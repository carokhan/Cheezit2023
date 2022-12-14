package frc.robot;

import java.util.List;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.*;
import frc.robot.commands.drivetrain.FollowTrajectory;
import frc.robot.commands.drivetrain.OperatorControl;
import frc.robot.commands.drivetrain.TestHeading;
import frc.robot.subsystems.DriveSubsystem;

public class RobotContainer {

    public static XboxController driveStick = new XboxController(ControlConstants.kDriverPort);

    private DriveSubsystem drive = new DriveSubsystem();
    
    public RobotContainer() {
        configureButtonBindings();
    }

    private void configureButtonBindings() {
        drive.setDefaultCommand(
            new OperatorControl(
                drive, 
                () -> driveStick.getLeftY(), 
                () -> driveStick.getLeftX(), 
                () -> driveStick.getRawAxis(3),
                true
            )
        );

        Trigger testHeading = new Trigger(drive::getSwerveTest);
        testHeading.onTrue(new TestHeading(drive, () -> drive.getTestHeading()));
    }

    public Command getAutonomousCommand() {

        TrajectoryConfig config = new TrajectoryConfig(
            AutoConstants.kMaxVelocity, 
            AutoConstants.kMaxAccel
        )
        .setKinematics(DriveConstants.kDriveKinematics);

        config.setStartVelocity(0.0);
        config.setEndVelocity(0.0);

        Trajectory autoNavSlalomTrajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(Units.inchesToMeters(48.5), Units.inchesToMeters(30), new Rotation2d(0)),
            List.of(new Translation2d(Units.inchesToMeters(80), Units.inchesToMeters(60)),
                    new Translation2d(Units.inchesToMeters(120), Units.inchesToMeters(90)),
                    new Translation2d(Units.inchesToMeters(240), Units.inchesToMeters(90)),
                    new Translation2d(Units.inchesToMeters(270), Units.inchesToMeters(60)),
                    new Translation2d(Units.inchesToMeters(300), Units.inchesToMeters(30)),
                    new Translation2d(Units.inchesToMeters(330), Units.inchesToMeters(60)),
                    new Translation2d(Units.inchesToMeters(300), Units.inchesToMeters(90)),
                    new Translation2d(Units.inchesToMeters(280), Units.inchesToMeters(60)),
                    new Translation2d(Units.inchesToMeters(240), Units.inchesToMeters(20)),
                    new Translation2d(Units.inchesToMeters(120), Units.inchesToMeters(20)),
                    new Translation2d(Units.inchesToMeters(90), Units.inchesToMeters(55))),
            new Pose2d(Units.inchesToMeters(48.5), Units.inchesToMeters(75), new Rotation2d(0)), config);

        FollowTrajectory runTrajectory = new FollowTrajectory(drive, autoNavSlalomTrajectory);

        return runTrajectory;
        
    }

}
