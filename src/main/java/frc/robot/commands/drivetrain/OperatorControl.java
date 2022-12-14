package frc.robot.commands.drivetrain;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.*;
import frc.robot.subsystems.DriveSubsystem;

public class OperatorControl extends CommandBase {

    private final DriveSubsystem drive;

    private final DoubleSupplier forwardX;
    private final DoubleSupplier forwardY;
    private final DoubleSupplier rotation;
    
    private final boolean isFieldRelative;

    public OperatorControl(
        DriveSubsystem subsystem, 
        DoubleSupplier fwdX, 
        DoubleSupplier fwdY, 
        DoubleSupplier rot,
        boolean fieldRelative
    ) {

        drive = subsystem;
        forwardX = fwdX;
        forwardY = fwdY;
        rotation = rot;

        isFieldRelative = fieldRelative;

        addRequirements(subsystem);

    }
    
    @Override
    public void execute() {

        double fwdX = forwardX.getAsDouble();
        fwdX = Math.copySign(Math.pow(fwdX, 2), fwdX);
        fwdX = deadbandInputs(fwdX);

        double fwdY = forwardY.getAsDouble();
        fwdY = Math.copySign(Math.pow(fwdY, 2), fwdY);
        fwdY = deadbandInputs(fwdY);

        double rot = rotation.getAsDouble();
        rot = Math.copySign(Math.pow(rot, 2), rot);
        rot = deadbandInputs(rot);

        drive.drive(
            fwdX,
            -fwdY,
            -rot,
            isFieldRelative
        );

    }

    public double deadbandInputs(double input) {
        if (Math.abs(input) < DriveConstants.kDeadband) return 0.0;
        return input;
    }

}