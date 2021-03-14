// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
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
  Pose2d pose = new Pose2d();

  WPI_TalonFX leftFront = new WPI_TalonFX(Constants.DriveTalonIDs.leftFrontID);
  WPI_TalonFX leftBack = new WPI_TalonFX(Constants.DriveTalonIDs.leftBackID);
  WPI_TalonFX rightFront = new WPI_TalonFX(Constants.DriveTalonIDs.rightFrontID);
  WPI_TalonFX rightBack = new WPI_TalonFX(Constants.DriveTalonIDs.rightBackID);

  SpeedControllerGroup leftDrive = new SpeedControllerGroup(leftFront, leftBack);
  SpeedControllerGroup rightDrive = new SpeedControllerGroup(rightFront, rightBack);

  DifferentialDrive diffDrive = new DifferentialDrive(leftDrive, rightDrive);
  DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Constants.PhysicalRobotConstants.kTrackWidthMeters);
  DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(getHeading(), pose); 

  SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.PhysicalRobotConstants.kS, Constants.PhysicalRobotConstants.kV, Constants.PhysicalRobotConstants.kA);

  PIDController leftPIDController = new PIDController(DriveConstants.kP, DriveConstants.kI, DriveConstants.kD);
  PIDController rightPIDController = new PIDController(DriveConstants.kP, DriveConstants.kI, DriveConstants.kD);

  RobotContainer container = new RobotContainer();


  
  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    //leftFront.setSensorPhase(true);
    //rightFront.setSensorPhase(true);
    //diffDrive.setSafetyEnabled(false);
    rightFront.setInverted(true);
    rightBack.setInverted(true);
  }


  public void drive(double speed, double rotation) {
    diffDrive.arcadeDrive(speed, rotation);
  }

  public boolean isMotorInverted() {
    //System.out.println("Is this motor inverted???? " + isMotorInverted() + " my Lord.");
    return rightBack.getInverted();
  }

  public void stop() {
    leftDrive.set(0);
    rightDrive.set(0);
  }

  public double getDistance() {
    return Constants.PhysicalRobotConstants.feetPerTick *(leftFront.getSelectedSensorPosition() - rightFront.getSelectedSensorPosition())/2;
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
  //changes

  public PIDController getLeftPIDController() {
    return leftPIDController;
  }
  public PIDController getRightPIDController() {
    return rightPIDController;
  }

  public TalonFX getLeftTalonFX() {
    return leftFront;
  }
  public TalonFX getRightTalonFX() {
    return rightFront;
  }

  public void setVoltageOutput(double leftVoltage, double rightVoltage) {
    leftFront.setVoltage(leftVoltage);
    rightFront.setVoltage(rightVoltage);
  }

  public void resetEncoders() {
    leftFront.setSelectedSensorPosition(0);
    rightFront.setSelectedSensorPosition(0);
  }

  public void resetHeading() {
    gyro.reset();
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    odometry.resetPosition(pose, gyro.getRotation2d());
  }

  public double getTurnRate() {
    return gyro.getRate();
  }
  
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    pose = odometry.update(getHeading(), getLeftMotorVelocity(), getRightMotorVelocity());
    //System.out.println("Is this motor inverted???? " + isMotorInverted() + " my Lord.");
    System.out.println("LeftMotorVelocity: " + getLeftMotorVelocity());
    System.out.println("RightMotorVelocity: " + getRightMotorVelocity());

    //robot went back and forth weirdly so that's why we did print statements
  }
} 
