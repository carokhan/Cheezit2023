package frc.robot.commands.drivetrain;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Constants.*;

public class TestHeading extends CommandBase {
    
    private final DriveSubsystem drive;
    private final DoubleSupplier rotation;
    
    public TestHeading(
        DriveSubsystem subsystem, 
        DoubleSupplier rot
    ) {

        drive = subsystem;
        rotation = rot;

        addRequirements(subsystem);

    }

    @Override
    public void execute() {

        double rot = rotation.getAsDouble();
        rot = Math.copySign(Math.pow(rot, 2), rot);
        rot = deadbandInputs(rot);

        drive.drive(0, 0, -rot,false);

    }

    public double deadbandInputs(double input) {
        if (Math.abs(input) < DriveConstants.kDeadband) return 0.0;
        return input;
    }
    
}
