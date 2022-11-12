package frc.robot.subsystems;

import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;

public class SwerveModule extends SubsystemBase {

    private final CANSparkMax driveMotor;
    private final CANSparkMax rotationMotor;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder rotationEncoder;

    private final DutyCycleEncoder absoluteEncoder;

    private final Rotation2d offset;

    private final SparkMaxPIDController rotationController;

    public SwerveModule(
        int driveMotorId, 
        int rotationMotorId,
        int absoluteChannel,
        double measuredOffsetDegrees
    ) {

        driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
        rotationMotor = new CANSparkMax(rotationMotorId, MotorType.kBrushless);

        driveEncoder = driveMotor.getEncoder();
        rotationEncoder = rotationMotor.getEncoder();

        absoluteEncoder = new DutyCycleEncoder(new DigitalInput(absoluteChannel));

        offset = new Rotation2d(Units.degreesToRadians(measuredOffsetDegrees));

        driveMotor.setIdleMode(IdleMode.kBrake);
        rotationMotor.setIdleMode(IdleMode.kCoast);

        rotationController = rotationMotor.getPIDController();

        rotationController.setP(DriveConstants.kPRotation);
        rotationController.setI(DriveConstants.kIRotation);
        rotationController.setD(DriveConstants.kDRotation);

        driveEncoder.setPositionConversionFactor(
            RobotMeasurements.WHEEL_CIRCUMFERENCE / RobotMeasurements.DRIVE_GEAR_RATIO
        );

        driveEncoder.setVelocityConversionFactor(
            RobotMeasurements.WHEEL_CIRCUMFERENCE / 60 / RobotMeasurements.DRIVE_GEAR_RATIO
        );

        rotationEncoder.setPositionConversionFactor(2 * Math.PI / RobotMeasurements.TURN_GEAR_RATIO);
    }

    public Rotation2d getAbsoluteAngle() {
        return new Rotation2d(Units.degreesToRadians(((absoluteEncoder.get() * 360.0) - offset.getDegrees()) % 360));
    }

    public Rotation2d getCanEncoderAngle() {

        double unsignedAngle = rotationEncoder.getPosition() % (2 * Math.PI);

        if (unsignedAngle < 0) unsignedAngle += 2 * Math.PI;

        return new Rotation2d(unsignedAngle);

    }

    public double getCurrentVelocity() {
        return driveEncoder.getVelocity();
    }

    public double calculateAdjustedAngle(double targetAngle, double currentAngle) {

        double modAngle = currentAngle % (2.0 * Math.PI);

        if (modAngle < 0.0) modAngle += 2.0 * Math.PI;
        
        double newTarget = targetAngle + currentAngle - modAngle;

        if (targetAngle - modAngle > Math.PI) newTarget -= 2.0 * Math.PI;
        else if (targetAngle - modAngle < -Math.PI) newTarget += 2.0 * Math.PI;

        return newTarget;

    }

    public void initRotationOffset() {
        rotationEncoder.setPosition(Units.degreesToRadians((absoluteEncoder.get() * 360.0) % 360));
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        
        SwerveModuleState state = SwerveModuleState.optimize(desiredState, getCanEncoderAngle());

        rotationController.setReference(
            calculateAdjustedAngle(
                state.angle.getRadians(), 
                rotationEncoder.getPosition()),
            ControlType.kPosition
        );

        driveMotor.set(state.speedMetersPerSecond);

    }

    public void resetEncoders() {
        driveEncoder.setPosition(0);
        rotationEncoder.setPosition(0);
    }
    
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(driveEncoder.getPosition(), Rotation2d.fromDegrees(Units.radiansToDegrees(rotationEncoder.getPosition())));
    }
};