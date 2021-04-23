// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.concurrent.TimeUnit;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.PhysicalRobotConstants;

public class DriveSubsystem extends SubsystemBase {
  

  AHRS gyro = new AHRS();
  //Pose2d pose = new Pose2d();

  WPI_TalonFX leftFront = new WPI_TalonFX(Constants.DriveTalonIDs.leftFrontID);
  WPI_TalonFX leftBack = new WPI_TalonFX(Constants.DriveTalonIDs.leftBackID);
  WPI_TalonFX rightFront = new WPI_TalonFX(Constants.DriveTalonIDs.rightFrontID);
  WPI_TalonFX rightBack = new WPI_TalonFX(Constants.DriveTalonIDs.rightBackID);

  SpeedControllerGroup leftDrive = new SpeedControllerGroup(leftFront, leftBack);
  SpeedControllerGroup rightDrive = new SpeedControllerGroup(rightFront, rightBack);

  DifferentialDrive diffDrive = new DifferentialDrive(leftDrive, rightDrive);
  DifferentialDriveKinematics kinematics = DriveConstants.kDriveKinematics;
  
  DifferentialDriveOdometry odometry;//new DifferentialDriveOdometry(getHeading(), pose); 

  SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.PhysicalRobotConstants.kS, Constants.PhysicalRobotConstants.kV, Constants.PhysicalRobotConstants.kA);

  PIDController leftPIDController = new PIDController(DriveConstants.kP, DriveConstants.kI, DriveConstants.kD);
  PIDController rightPIDController = new PIDController(DriveConstants.kP, DriveConstants.kI, DriveConstants.kD);

  static double yawError = 0;
  static Rotation2d idealRotation2d;



  
  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    //leftFront.setSensorPhase(true);
    //rightFront.setSensorPhase(true);
    //diffDrive.setSafetyEnabled(true);
    //rightFront.setInverted(true);
    //rightBack.setInverted(true);
    System.out.println("Drive system initializing!");
    resetEncoders();
    odometry = new DifferentialDriveOdometry(gyro.getRotation2d()); 
  }


  public void drive(double speed, double rotation) {
    diffDrive.arcadeDrive(speed, rotation);
  }

  /*public boolean isMotorInverted() {
    //System.out.println("Is this motor inverted???? " + isMotorInverted() + " my Lord.");
    return rightBack.getInverted();
  }*/

  public void stop() {
    leftDrive.set(0);
    rightDrive.set(0);
  }

  public double getDistance() {
    return Constants.PhysicalRobotConstants.feetPerTick *(leftFront.getSelectedSensorPosition() - rightFront.getSelectedSensorPosition())/2;
  }

  public double nativeUnitsToDistanceMeters(double ticks) {
    double motorRotations = ticks / PhysicalRobotConstants.kEncoderCPR;
    double wheelRotations = motorRotations / PhysicalRobotConstants.kGearRatio;
    double positionMeters = wheelRotations * (Math.PI * PhysicalRobotConstants.kWheelDiameterMeters);
    return positionMeters;
  }
  
  /* public double getMotorVeloctyLeft() {
    return leftFront.getSelectedSensorVelocity();
  }
  public double getMotorVeloctyRight() {
    return rightFront.getSelectedSensorVelocity();
  } */

  public Pose2d getPose() {
    System.out.println("Current Robot Pose::: " + odometry.getPoseMeters());
    return odometry.getPoseMeters();
  }

  public Rotation2d getHeading(){
    idealRotation2d = gyro.getRotation2d().fromDegrees(yawError);
    System.out.println("Current Heading::: " + gyro.getRotation2d());
    return idealRotation2d;
    //gyroscopes return positive values for clockwise but in standard convention it's the opposite(how the unit circle works)
    
  }

  public DifferentialDriveKinematics getKinematics() {
    return kinematics;
    
  }

  public double getLeftMotorVelocity() {
    double ticksPerSec = leftFront.getSelectedSensorVelocity() * 10;
    double metersPerSec = PhysicalRobotConstants.metersPerTick * ticksPerSec;
    return metersPerSec;

    /*double distanceMeters = nativeUnitsToDistanceMeters(leftFront.getSelectedSensorPosition()); //runs every .02 sec
    System.out.println("Distance in Meters::::::: " + distanceMeters);
    System.out.println("Velocity in Meters/Sec::::::: " + distanceMeters/.02);
    return distanceMeters/.02; */
    // .distanceMe
    
    //return leftFront.getSelectedSensorVelocity() / 1861; //ticks/100ms * () -> m/sec
    //need to convert from raw sensor units to meters per second
  }

  public double getRightMotorVelocity() {
    double ticksPerSec = rightFront.getSelectedSensorVelocity() * 10;
    double metersPerSec = PhysicalRobotConstants.metersPerTick * ticksPerSec;
    return metersPerSec;
    /*double distanceMeters = nativeUnitsToDistanceMeters(rightFront.getSelectedSensorPosition()); //runs every .02 sec
    System.out.println("Distance in Meters::::::: " + distanceMeters);
    System.out.println("Velocity in Meters/Sec::::::: " + distanceMeters/.02);
    return distanceMeters/.02;*/
    //return rightFront.getSelectedSensorVelocity() / 1861;
    //need to convert from raw sensor units/100ms to meters per second
    // 4096 ticks/
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getLeftMotorVelocity(), getRightMotorVelocity());
  }

  public SimpleMotorFeedforward getFeedForward() {
    return feedforward;
  }

  public double getHeadingDegrees() {
    return gyro.getRotation2d().getDegrees();
  }

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
    diffDrive.feed();
  }

  public void resetEncoders() {
    leftFront.setSelectedSensorPosition(0);
    rightFront.setSelectedSensorPosition(0);
  }

  public void resetHeading() {
    System.out.println("Resetting the yaw value of the gyro!");
    gyro.reset();

    while(gyro.isCalibrating()) {
      try {
        System.out.println("Gyro is calibrating!");
        Thread.sleep(200);
      } catch(InterruptedException exception) {
        Thread.currentThread().interrupt();
      }
      yawError = 0;
    }

    System.out.println("Initial Heading Values::: " + gyro.getRotation2d());
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    gyro.reset();
    System.out.println("Heading before:: " + gyro.getRotation2d());
    odometry.resetPosition(pose, gyro.getRotation2d());
    System.out.println("Heading after:: " + gyro.getRotation2d());
  }

  public double getTurnRate() {
    return -gyro.getRate();
  }
  
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    /*if (yawError == 0) {
      yawError = gyro.getYaw();
      System.out.println("YAW ERROR::: " + yawError);
    }*/
    //System.out.println("PRE-PERIODIC HEADING::::" + gyro.getRotation2d());
    odometry.update(getHeading(), getLeftMotorVelocity(), getRightMotorVelocity());
    //System.out.println("POST-PERIODIC HEADING::::" + gyro.getRotation2d());
    //System.out.println("Is this motor inverted???? " + isMotorInverted() + " my Lord.");
    //System.out.println("LeftMotorVelocity: " + getLeftMotorVelocity());
    //System.out.println("RightMotorVelocity: " + getRightMotorVelocity());

  }
} 
