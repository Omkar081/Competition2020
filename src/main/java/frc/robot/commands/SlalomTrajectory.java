// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.PhysicalRobotConstants;
import frc.robot.subsystems.DriveSubsystem;

public class SlalomTrajectory extends CommandBase {
  DriveSubsystem drive;

  DifferentialDriveVoltageConstraint autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(PhysicalRobotConstants.kS, PhysicalRobotConstants.kV, PhysicalRobotConstants.kA),
            DriveConstants.kDriveKinematics,
            PhysicalRobotConstants.kMaxVoltage);


  TrajectoryConfig config =
    new TrajectoryConfig(DriveConstants.maxV, DriveConstants.maxA)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics)
        // Apply the voltage constraint
        .addConstraint(autoVoltageConstraint);

  Trajectory slalomtrajectory = TrajectoryGenerator.generateTrajectory(
    new Pose2d(0, 0, new Rotation2d(0)),
    List.of(
      new Translation2d(Units.feetToMeters(2.5), Units.feetToMeters(2.5)),
      new Translation2d(Units.feetToMeters(5), Units.feetToMeters(5)),
      new Translation2d(Units.feetToMeters(10), Units.feetToMeters(5.5)),
      new Translation2d(Units.feetToMeters(15), Units.feetToMeters(5)),
      new Translation2d(Units.feetToMeters(17.5), Units.feetToMeters(2.5)),
      new Translation2d(Units.feetToMeters(20), Units.feetToMeters(0)),
      new Translation2d(Units.feetToMeters(22.5), Units.feetToMeters(2.5)),
      new Translation2d(Units.feetToMeters(20), Units.feetToMeters(5)),
      new Translation2d(Units.feetToMeters(15), Units.feetToMeters(0)),
      new Translation2d(Units.feetToMeters(10), Units.feetToMeters(0)),
      new Translation2d(Units.feetToMeters(5), Units.feetToMeters(0))
    ),
    new Pose2d(Units.feetToMeters(0), Units.feetToMeters(5), new Rotation2d(Math.PI)),
    config
    ); 

    RamseteCommand ramseteCommand = new RamseteCommand(
      slalomtrajectory, 
      drive::getPose, 
      new RamseteController(2.0, 0.7), 
      new SimpleMotorFeedforward(PhysicalRobotConstants.kS, PhysicalRobotConstants.kV, PhysicalRobotConstants.kA), 
      DriveConstants.kDriveKinematics, 
      drive::getWheelSpeeds, 
      new PIDController(DriveConstants.kP, DriveConstants.kI, DriveConstants.kD), 
      new PIDController(DriveConstants.kP, DriveConstants.kI, DriveConstants.kD), 
      drive::setVoltageOutput, 
      drive
      );
    
  
  /** Creates a new SlalomTrajectory. */
  public SlalomTrajectory(DriveSubsystem drive) {
    this.drive = drive;
    //ramseteCommand.schedule();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
    ramseteCommand.schedule();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.resetEncoders();
    drive.resetHeading();
    drive.resetOdometry(slalomtrajectory.getInitialPose());
    config.setKinematics(drive.getKinematics());
    ramseteCommand.initialize();
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
