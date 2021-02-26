// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.util.Units;

/** Add your docs here. */
public class Trajectories {

    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
        Units.feetToMeters(Constants.DriveConstants.maxV), 
        Units.feetToMeters(Constants.DriveConstants.maxA)
    );
    
    public final class SlalomTrajectories{
        public final Trajectory slalomtrajectory1 = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)),
            List.of(
                new Translation2d(Units.feetToMeters(2.5), Units.feetToMeters(2.5)),
                new Translation2d(Units.feetToMeters(5), Units.feetToMeters(5)),
                new Translation2d(Units.feetToMeters(10), Units.feetToMeters(5.5)),
                new Translation2d(Units.feetToMeters(15), Units.feetToMeters(5)),
                new Translation2d(Units.feetToMeters(17.5), Units.feetToMeters(2.5)),
                new Translation2d(Units.feetToMeters(20), Units.feetToMeters(0))
            ),
            new Pose2d(Units.feetToMeters(22.5), Units.feetToMeters(2.5), new Rotation2d(Math.PI/2)),
            trajectoryConfig
        );

        public final Trajectory slalomtrajectory2 = TrajectoryGenerator.generateTrajectory(
            new Pose2d(22.5, 2.5, new Rotation2d(Math.PI/2)),
            List.of(
                new Translation2d(Units.feetToMeters(20), Units.feetToMeters(5)),
                new Translation2d(Units.feetToMeters(17.5), Units.feetToMeters(2.5)),
                new Translation2d(Units.feetToMeters(15), Units.feetToMeters(0)),
                new Translation2d(Units.feetToMeters(10), Units.feetToMeters(-0.5)),
                new Translation2d(Units.feetToMeters(5), Units.feetToMeters(0)),
                new Translation2d(Units.feetToMeters(2.5), Units.feetToMeters(2.5))
            ),
            new Pose2d(Units.feetToMeters(0), Units.feetToMeters(5), new Rotation2d(Math.PI)),
            trajectoryConfig
        );
    }

    public final class BounceTrajectories{
        public final Trajectory bounceTrajectory1 = TrajectoryGenerator.generateTrajectory(
            new Pose2d(2.5, 5, new Rotation2d(Math.PI + Math.atan(2))),
            List.of(
                new Translation2d(Units.feetToMeters(3), Units.feetToMeters(0)),
                new Translation2d(Units.feetToMeters(5.5), Units.feetToMeters(-3.5)),
                new Translation2d(Units.feetToMeters(7.5), Units.feetToMeters(-5)),
                new Translation2d(Units.feetToMeters(9.25), Units.feetToMeters(-3.5))
                //new Translation2d(Units.feetToMeters(10), Units.feetToMeters(7.5))
                //new Translation2d(Units.feetToMeters(2.5), Units.feetToMeters(2.5))
            ),
            new Pose2d(Units.feetToMeters(10), Units.feetToMeters(7.5), new Rotation2d(Math.PI/2)),
            trajectoryConfig
        );

        /*public final Trajectory bounceTrajectory2 = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(Math.PI/2),
            List.of(
                new Translation2d(Units.feetToMeters(1), Units.feetToMeters(-5))
                new Translation2d(Units.feetToMeters(5.5), Units.feetToMeters(-3.5)),
                new Translation2d(Units.feetToMeters(7.5), Units.feetToMeters(-5)),
                new Translation2d(Units.feetToMeters(9.25), Units.feetToMeters(-3.5)),
                new Translation2d(Units.feetToMeters(10), Units.feetToMeters(7.5))
                //new Translation2d(Units.feetToMeters(2.5), Units.feetToMeters(2.5))
            ),
            new Pose2d(Units.feetToMeters(0), Units.feetToMeters(5), new Rotation2d(Math.PI)),
            trajectoryConfig
        );*/
        

        

    }

    public final class BarrelRacingTrajectories{

    }
}
