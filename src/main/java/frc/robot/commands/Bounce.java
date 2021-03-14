// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class Bounce extends CommandBase {
  DriveSubsystem drive;

  TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
    Units.feetToMeters(Constants.DriveConstants.maxV), 
    Units.feetToMeters(Constants.DriveConstants.maxA)
    );

  Trajectory bounce = TrajectoryGenerator.generateTrajectory(
    new Pose2d(0, 0, new Rotation2d(0)), //start
    List.of(
      new Translation2d(Units.feetToMeters(1.5), Units.feetToMeters(2.5)),
      new Translation2d(Units.feetToMeters(2.5), Units.feetToMeters(5)),
      new Translation2d(Units.feetToMeters(3.5), Units.feetToMeters(0)),
      new Translation2d(Units.feetToMeters(7.5), Units.feetToMeters(-5)),
      new Translation2d(Units.feetToMeters(10), Units.feetToMeters(5)),
      new Translation2d(Units.feetToMeters(12.5), Units.feetToMeters(-5)),
      new Translation2d(Units.feetToMeters(16.25), Units.feetToMeters(-4)),
      new Translation2d(Units.feetToMeters(17.5), Units.feetToMeters(5))
    ),
    new Pose2d(Units.feetToMeters(0), Units.feetToMeters(2), new Rotation2d(Math.PI)), //end
    trajectoryConfig
    );

    RamseteCommand ramseteCommand = new RamseteCommand(
      bounce, 
      drive::getRobotPose, 
      new RamseteController(2.0, 0.7),
      drive.getFeedForward(), 
      drive.getKinematics(), 
      drive::getWheelSpeeds, 
      drive.getLeftPIDController(), 
      drive.getRightPIDController(), 
      drive::setVoltageOutput, 
      drive
    );
  
  /** Creates a new BarrelRacing. */
  public Bounce(DriveSubsystem drive) {
    this.drive = drive;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.resetEncoders();
    drive.resetHeading();
    trajectoryConfig.setKinematics(drive.getKinematics());
    bounce.getInitialPose();
  
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ramseteCommand.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ramseteCommand.end(interrupted);
    drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ramseteCommand.isFinished();
  }
}
