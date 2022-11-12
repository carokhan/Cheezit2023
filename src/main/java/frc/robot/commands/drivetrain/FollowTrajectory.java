package frc.robot.commands.drivetrain;

import edu.wpi.first.math.trajectory.Trajectory;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.*;
import frc.robot.subsystems.DriveSubsystem;

public class FollowTrajectory extends CommandBase {

    private final Trajectory trajectory;

    private Timer time = new Timer();

    private final SwerveDriveKinematics kinematics = DriveConstants.kDriveKinematics;

    private DriveSubsystem drive;

    private static final ProfiledPIDController rotationController = 
        new ProfiledPIDController(AutoConstants.kPTurn, AutoConstants.kITurn, AutoConstants.kDTurn,
            new TrapezoidProfile.Constraints(
                AutoConstants.kMaxVelocity,
                AutoConstants.kMaxAccel
            )
        );

    private static final HolonomicDriveController controller = 
        new HolonomicDriveController(
            new PIDController(AutoConstants.kPDrive, AutoConstants.kIDrive, AutoConstants.KDDrive), 
            new PIDController(AutoConstants.kPDrive, AutoConstants.kIDrive, AutoConstants.KDDrive), 
            rotationController
        );

    public FollowTrajectory(DriveSubsystem drives, Trajectory trajectory) {
        this.trajectory = trajectory;
        drive = drives;
        controller.setTolerance(AutoConstants.kTolerances);

        rotationController.enableContinuousInput(-Math.PI, Math.PI);
    }
    
    public Pose2d getInitialPose() {
        return trajectory.getInitialPose();
    }

    public Pose2d getCurrentPose() {
        return drive.getPose();
    }

    @Override
    public void initialize() {
        time.reset();
        time.start();
    }

    @Override
    public void execute() {

        Trajectory.State goal = trajectory.sample(time.get());

        SmartDashboard.putString("X wanted", " " + goal.poseMeters.getX()); 
        SmartDashboard.putString("Y wanted", " " + goal.poseMeters.getY()); 

        ChassisSpeeds adjustedSpeeds = 
            controller.calculate(
                getCurrentPose(), 
                goal, 
                Rotation2d.fromDegrees(0.0)
            );

        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(adjustedSpeeds);

        SmartDashboard.putString("Actual speed", " " + (drive.getModuleStates())[1].speedMetersPerSecond);
        SmartDashboard.putString("Speed wanted", " " + moduleStates[1].speedMetersPerSecond);     
        
        drive.setModuleStates(moduleStates);

    }

    @Override
    public void end(boolean interrupted) {
        time.stop();
    }

    @Override
    public boolean isFinished() {
        return time.hasElapsed(trajectory.getTotalTimeSeconds());
    }

}