// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.util.List;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
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
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.PhysicalRobotConstants;
import frc.robot.commands.BarrelRacing;
import frc.robot.commands.Bounce;
import frc.robot.commands.Drive;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.SlalomTrajectory;
//import frc.robot.commands.SlalomTrajectory1;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  Joystick joystick = new Joystick(0);
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();

  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);
  private final Drive m_drive = new Drive(() -> joystick.getY(), () -> joystick.getX(), m_driveSubsystem);
  /*private final SlalomTrajectory slalomTrajectory = new SlalomTrajectory(m_driveSubsystem);
  private final BarrelRacing barrelRacing = new BarrelRacing(m_driveSubsystem);
  private final Bounce bounce = new Bounce(m_driveSubsystem);*/

  /*TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
    Units.feetToMeters(Constants.DriveConstants.maxV), 
    Units.feetToMeters(Constants.DriveConstants.maxA)
    );*/

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    m_driveSubsystem.setDefaultCommand(m_drive);
    //trajectoryConfig.setKinematics(m_driveSubsystem.getKinematics());
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    //return m_autoCommand;
    // Create a voltage constraint to ensure we don't accelerate too fast
    var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(PhysicalRobotConstants.kS, PhysicalRobotConstants.kV, PhysicalRobotConstants.kA),
            DriveConstants.kDriveKinematics,
            PhysicalRobotConstants.kMaxVoltage);

      // Create config for trajectory
    TrajectoryConfig config =
    new TrajectoryConfig(DriveConstants.maxV, DriveConstants.maxA)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics)
        // Apply the voltage constraint
        .addConstraint(autoVoltageConstraint);

    Trajectory slalomtrajectory = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0, 0, new Rotation2d(0)),
      List.of(
        new Translation2d(Units.feetToMeters(5), Units.feetToMeters(0))
        /*new Translation2d(Units.feetToMeters(2.5), Units.feetToMeters(2.5)),
        new Translation2d(Units.feetToMeters(5), Units.feetToMeters(5)),
        new Translation2d(Units.feetToMeters(10), Units.feetToMeters(5.5)),
        new Translation2d(Units.feetToMeters(15), Units.feetToMeters(5)),
        new Translation2d(Units.feetToMeters(17.5), Units.feetToMeters(2.5)),
        new Translation2d(Units.feetToMeters(20), Units.feetToMeters(0)),
        new Translation2d(Units.feetToMeters(22.5), Units.feetToMeters(2.5)),
        new Translation2d(Units.feetToMeters(20), Units.feetToMeters(5)),
        new Translation2d(Units.feetToMeters(15), Units.feetToMeters(0)),
        new Translation2d(Units.feetToMeters(10), Units.feetToMeters(0)),
        new Translation2d(Units.feetToMeters(5), Units.feetToMeters(0)) */
      ),
      new Pose2d(Units.feetToMeters(10), Units.feetToMeters(0), new Rotation2d(Math.PI/2)),
      config
      );

      RamseteCommand ramseteCommand = new RamseteCommand(
        slalomtrajectory, 
        m_driveSubsystem::getRobotPose, 
        new RamseteController(2.0, 0.7), 
        new SimpleMotorFeedforward(PhysicalRobotConstants.kS, PhysicalRobotConstants.kV, PhysicalRobotConstants.kA), 
        DriveConstants.kDriveKinematics, 
        m_driveSubsystem::getWheelSpeeds, 
        new PIDController(DriveConstants.kP, DriveConstants.kI, DriveConstants.kD), 
        new PIDController(DriveConstants.kP, DriveConstants.kI, DriveConstants.kD), 
        m_driveSubsystem::setVoltageOutput, 
        m_driveSubsystem
        );
      //Reset odometry to the starting pose of the trajectory
      m_driveSubsystem.resetOdometry(slalomtrajectory.getInitialPose());

      //Run path following command, then stop at the end
      return ramseteCommand.andThen(() -> m_driveSubsystem.setVoltageOutput(0, 0));
  }
}
