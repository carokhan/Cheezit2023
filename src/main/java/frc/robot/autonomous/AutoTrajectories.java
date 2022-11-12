
package frc.robot.autonomous;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.*;

public class AutoTrajectories {

    private static TrajectoryConfig config;

    public static Trajectory autoNavSlalomTrajectory;

    public AutoTrajectories() {

        config = new TrajectoryConfig(
            AutoConstants.kMaxVelocity, 
            AutoConstants.kMaxAccel
        )
        .setKinematics(DriveConstants.kDriveKinematics);

        config.setStartVelocity(0.0);
        config.setEndVelocity(0.0);

        autoNavSlalomTrajectory = TrajectoryGenerator.generateTrajectory(
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

    }

}