package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Map;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants.*;

public class DriveSubsystem extends SubsystemBase {

        // FIXME: switch to array of modules like Spectrum/Flash-2023 for cool loops?
        private final SwerveModule frontLeft = 
            new SwerveModule(
                RobotMap.FRONT_LEFT_DRIVE,
                RobotMap.FRONT_LEFT_TURN,
                RobotMap.FRONT_LEFT_ENCODER,
                RobotMap.FRONT_LEFT_OFFSET
            );

        private final SwerveModule frontRight = 
            new SwerveModule(
                RobotMap.FRONT_RIGHT_DRIVE,
                RobotMap.FRONT_RIGHT_TURN,
                RobotMap.FRONT_RIGHT_ENCODER,
                RobotMap.FRONT_RIGHT_OFFSET
            );

        private final SwerveModule rearLeft = 
            new SwerveModule(
                RobotMap.REAR_LEFT_DRIVE,
                RobotMap.REAR_LEFT_TURN,
                RobotMap.REAR_LEFT_ENCODER,
                RobotMap.REAR_LEFT_OFFSET
            );

        private final SwerveModule rearRight = 
            new SwerveModule(
                RobotMap.REAR_RIGHT_DRIVE,
                RobotMap.REAR_RIGHT_TURN,
                RobotMap.REAR_RIGHT_ENCODER,
                RobotMap.REAR_RIGHT_OFFSET
            );

    private final AHRS navx = new AHRS();

    private final SwerveDriveOdometry odometry = 
        new SwerveDriveOdometry(
            DriveConstants.kDriveKinematics, 
            navx.getRotation2d(),
            getModulePositions()
        );

    private ShuffleboardTab sboard = Shuffleboard.getTab("Dashboard");
    private GenericEntry swerveTest = sboard.add("Test Heading", false).withWidget(BuiltInWidgets.kToggleButton).getEntry();
    private GenericEntry testHeading = sboard.add("Heading", 0).withWidget(BuiltInWidgets.kNumberBar).withProperties(Map.of("min", 0, "max", 360)).getEntry();

    public DriveSubsystem() {
        navx.calibrate();

        frontLeft.initRotationOffset();
        frontRight.initRotationOffset();
        rearLeft.initRotationOffset();
        rearRight.initRotationOffset();
    }

    @Override
    public void periodic() {
        odometry.update(getHeading(), getModulePositions());
    }
    
    public void drive(double forward, double strafe, double rotation, boolean isFieldRelative) {
        
        ChassisSpeeds speeds =
            isFieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    forward, strafe, rotation, Rotation2d.fromDegrees(navx.getAngle()))
                : new ChassisSpeeds(forward, strafe, rotation);
        
        SwerveModuleState[] states = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);

        setModuleStates(states);

    }

    public void setModuleStates(SwerveModuleState[] moduleStates) {

        frontLeft.setDesiredState(moduleStates[2]);
        frontRight.setDesiredState(moduleStates[0]);
        rearLeft.setDesiredState(moduleStates[3]);
        rearRight.setDesiredState(moduleStates[1]);

    }

    public SwerveModuleState[] getModuleStates() {
        
        SwerveModuleState[] states = {
            new SwerveModuleState(frontRight.getCurrentVelocity(), frontRight.getCanEncoderAngle()),
            new SwerveModuleState(rearRight.getCurrentVelocity(), rearRight.getCanEncoderAngle()),
            new SwerveModuleState(frontLeft.getCurrentVelocity(), frontLeft.getCanEncoderAngle()),
            new SwerveModuleState(rearLeft.getCurrentVelocity(), rearLeft.getCanEncoderAngle())
        };

        return states;

    }

    public SwerveModulePosition[] getModulePositions() {

        SwerveModulePosition[] positions = {
            frontRight.getPosition(),
            rearRight.getPosition(),
            frontLeft.getPosition(),
            rearLeft.getPosition(),
        };

        return positions;

    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public void resetPose(Pose2d pose) {
        navx.reset();
        odometry.resetPosition(pose, Rotation2d.fromDegrees(-navx.getAngle()));
    }

    public Rotation2d getHeading() {
        return Rotation2d.fromDegrees(-navx.getAngle());
    }

    public boolean getSwerveTest() {
        return swerveTest.getBoolean(false);
    }

    public double getTestHeading() {
        return testHeading.getDouble(0);
    }
}