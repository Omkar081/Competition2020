// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {
  

  AHRS gyro = new AHRS();
  Pose2d pose;

  WPI_TalonSRX leftFront = new WPI_TalonSRX(Constants.DriveTalonIDs.leftFrontID);
  WPI_TalonSRX leftBack = new WPI_TalonSRX(Constants.DriveTalonIDs.leftBackID);
  WPI_TalonSRX rightFront = new WPI_TalonSRX(Constants.DriveTalonIDs.rightFrontID);
  WPI_TalonSRX rightBack = new WPI_TalonSRX(Constants.DriveTalonIDs.rightBackID);

  SpeedControllerGroup leftDrive = new SpeedControllerGroup(leftFront, leftBack);
  SpeedControllerGroup rightDrive = new SpeedControllerGroup(rightFront, rightBack);

  DifferentialDrive diffDrive = new DifferentialDrive(leftDrive, rightDrive);
  DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(Constants.PhysicalRobotConstants.trackWidthInches));
  DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(getHeading(),pose); 

  SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.PhysicalRobotConstants.kS, Constants.PhysicalRobotConstants.kV, Constants.PhysicalRobotConstants.kA);

  PIDController leftPIDController = new PIDController(DriveConstants.kP, DriveConstants.kI, DriveConstants.kD);
  PIDController rightPIDController = new PIDController(DriveConstants.kP, DriveConstants.kI, DriveConstants.kD);

  RobotContainer container = new RobotContainer();


  
  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {}


  public void drive(double speed, double rotation) {
    diffDrive.arcadeDrive(speed, rotation);
  }

  public void stop() {
    leftDrive.set(0);
    rightDrive.set(0);
  }

  public void resetEncoders() {
    //leftFront.reset();
    //rightFront.reset();
  }

  public Pose2d getRobotPose() {
    return pose;
  }

  public Rotation2d getHeading(){
    return Rotation2d.fromDegrees(-gyro.getAngle()); 
    //gyroscopes return positive values for clockwise but in standard convention it's the opposite(how the unit circle works)
    
  }

  public DifferentialDriveKinematics getKinematics() {
    return kinematics;
  }

  public double getLeftMotorVelocity() {
    return leftFront.getSelectedSensorVelocity();
    //need to convert from raw sensor units to meters per second
  }

  public double getRightMotorVelocity() {
    return rightFront.getSelectedSensorVelocity();
    //need to convert from raw sensor units to meters per second
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getLeftMotorVelocity(), getRightMotorVelocity());
  }

  public SimpleMotorFeedforward getFeedForward() {
    return feedforward;
  }

  public PIDController getLeftPIDController() {
    return leftPIDController;
  }
  public PIDController getRightPIDController() {
    return rightPIDController;
  }

  public void setVoltageOutput(double leftVoltage, double rightVoltage) {
    leftFront.setVoltage(leftVoltage);
    rightFront.setVoltage(rightVoltage);
  }

 
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    pose = odometry.update(getHeading(), getLeftMotorVelocity(), getRightMotorVelocity());
  }
} 
