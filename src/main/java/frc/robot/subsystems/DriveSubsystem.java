// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


//import java.sql.Time;
//import java.util.Timer;
//import java.time;

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

/**
 * This is the Drive Subsystem of our Robot. It contains some essential methods like drive, stop, getDistance, etc.
 * @author Zachary Hansen Terry and Pranav Vogeti
 */
public class DriveSubsystem extends SubsystemBase {
  
 Thread outputThread = new Thread();

  AHRS gyro = new AHRS();
  //Pose2d pose = new Pose2d();

  WPI_TalonFX leftFront = new WPI_TalonFX(Constants.DriveTalonIDs.leftFrontID);         // Creates the physical left front Talon motor as an object in the code
  WPI_TalonFX leftBack = new WPI_TalonFX(Constants.DriveTalonIDs.leftBackID);           // Creates the physical left back Talon Motor in the code
  WPI_TalonFX rightFront = new WPI_TalonFX(Constants.DriveTalonIDs.rightFrontID);       // Creates the right front motor on the robot in the code
  WPI_TalonFX rightBack = new WPI_TalonFX(Constants.DriveTalonIDs.rightBackID);         // Creates right rear motor in the code.
 

  SpeedControllerGroup leftDrive = new SpeedControllerGroup(leftFront, leftBack);       // Makes a group of the two left motors that can be treated similarly to one motor
  SpeedControllerGroup rightDrive = new SpeedControllerGroup(rightFront, rightBack);    // Maes a group of the motors on the right of the robot that have similar propertirs to a motor object

  DifferentialDrive diffDrive = new DifferentialDrive(leftDrive, rightDrive);           // Makes a grouping of all the motors together that can be controlled like a motor
  DifferentialDriveKinematics kinematics = DriveConstants.kDriveKinematics;             // Honestly IDK ask pranav
  
  DifferentialDriveOdometry odometry;       // Makes a odometer that allows us to track the robot on the field.

  SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.PhysicalRobotConstants.kS, Constants.PhysicalRobotConstants.kV, Constants.PhysicalRobotConstants.kA);

  PIDController leftPIDController = new PIDController(DriveConstants.kP, DriveConstants.kI, DriveConstants.kD); 
  PIDController rightPIDController = new PIDController(DriveConstants.kP, DriveConstants.kI, DriveConstants.kD);

 // static double yawError = 0;
  //static Rotation2d idealRotation2d;



  
  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    System.out.println("Drive system initializing!");
    resetEncoders();
    odometry = new DifferentialDriveOdometry(gyro.getRotation2d()); 
  }


  public void drive(double speed, double rotation) {
    diffDrive.arcadeDrive(speed, rotation);
  }

  /**
   * Stops all of the motors
   */
  public void stop() {
    leftDrive.set(0);
    rightDrive.set(0);
  }

  public double getDistance() {
    return Constants.PhysicalRobotConstants.feetPerTick *(leftFront.getSelectedSensorPosition() - rightFront.getSelectedSensorPosition())/2;
  }

  /**
   * Translates the ticks that the encoders provide to meters that we can use.
   * @param ticks
   * @return distance in meters
   */
  public double nativeUnitsToDistanceMeters(double ticks) {
    return (PhysicalRobotConstants.feetPerTick * ticks) / 3.28;
    /*double motorRotations = ticks / PhysicalRobotConstants.kFalconCPR;
    double wheelRotations = motorRotations / PhysicalRobotConstants.kGearRatio;
    double positionMeters = wheelRotations * (Math.PI * PhysicalRobotConstants.kWheelDiameterMeters);
    return positionMeters;*/
  }

  /**
   * Translates the encoders native units of ticks per 100ms to meters per second
   * @param ticksPerDecisecond
   * @return Meters per second
   */
  public double nativeUnitsToMetersPerSec(double ticksPerDeciSecond) {
    double ticksPerSec = ticksPerDeciSecond * 10; 
    return (ticksPerSec * PhysicalRobotConstants.feetPerTick)/ 3.28;

    /*double motorRotationsPerSec = ticksPerSec / PhysicalRobotConstants.kFalconCPR;
    double wheelRotationsPerSec = motorRotationsPerSec / PhysicalRobotConstants.kGearRatio;
    double metersPerSec = wheelRotationsPerSec *(PhysicalRobotConstants.kWheelDiameterMeters * Math.PI);
    return metersPerSec;*/
  }
  

  /**
   * Tells us the position in meters of the robot.
   * @return position in meters
   */
  public Pose2d getPose() {
    System.out.println("Current Robot Pose::: " + odometry.getPoseMeters());
    return odometry.getPoseMeters();
  }

  /**
   * Tells us the heading of the robot (yaw)
   * @return heading in degrees
   */
  public double getHeading(){
    return gyro.getRotation2d().getDegrees();
    /*idealRotation2d = gyro.getRotation2d().fromDegrees(yawError);
    System.out.println("Current Heading::: " + gyro.getRotation2d());
    return idealRotation2d; */
    //gyroscopes return positive values for clockwise but in standard convention it's the opposite(how the unit circle works)
    
  }

  public DifferentialDriveKinematics getKinematics() {
    return kinematics;
  }

  /**
   * Tells us the distance that the motor has traveled
   * @return Meters that the left motor has traveled
   * @see nativeUnitsToDistanceMeters
   * {@link #nativeUnitsToDistanceMeters(double)}
   */
  public double getLeftDistanceMeters() {
    return nativeUnitsToDistanceMeters((double) leftFront.getSelectedSensorPosition());
  }

/**
   * Tells us the distance that the motor has traveled
   * @return Meters that the right motor has traveled
   * @see nativeUnitsToDistanceMeters
   * {@link #nativeUnitsToDistanceMeters(double)}
   */
  public double getRightDistanceMeters() {
    return nativeUnitsToDistanceMeters((double) rightFront.getSelectedSensorPosition());
  }

  /**
   * Tells us the velocity of the left motor in meters per second.
   * @return m/s
   * @see
   * {@link #nativeUnitsToMetersPerSec(double)}
   */
  public double getLeftMotorVelocity() {
   //System.out.println("VELOCITY OF LEFT In m/s:::::: " + nativeUnitsToMetersPerSec(leftFront.getSelectedSensorVelocity()));
   return nativeUnitsToMetersPerSec((double)leftFront.getSelectedSensorVelocity());

  }

  /**
   * Tells us the velocity of the right motor in meters per second/
   * @return m/s
   * @see
   * {@link #nativeUnitsToMetersPerSec(double)}
   */
  public double getRightMotorVelocity() {
  //  System.out.println("VELOCITY OF RIGHT In m/s:::::: " + nativeUnitsToMetersPerSec(rightFront.getSelectedSensorVelocity()));
    return nativeUnitsToMetersPerSec((double)rightFront.getSelectedSensorVelocity());
  }

 
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getLeftMotorVelocity(), getRightMotorVelocity());
  }

  public SimpleMotorFeedforward getFeedForward() {
    return feedforward;
  }

  /**
   * I don't know if we ever use this in the code but I'm too scared to remove it so good luck(probably removable but I dont want to find out).
   * @see
   * {@link #getHeading()}
   */
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

  /**
   * Sets the voltage output for the left and right motors.
   * @param leftVoltage
   * @param rightVoltage
   */
  public void setVoltageOutput(double leftVoltage, double rightVoltage) {
    leftFront.setVoltage(leftVoltage);
    rightFront.setVoltage(-rightVoltage);
    diffDrive.feed();
  }

  /**Resets encoders for left and right motors.*/
  public void resetEncoders() {
    leftFront.setSelectedSensorPosition(0);
    rightFront.setSelectedSensorPosition(0);
  }

  /**Resets the yaw value of the gyro.*/
  public void resetHeading() {
    System.out.println("Resetting the yaw value of the gyro!");
    gyro.reset();

    /*while(gyro.isCalibrating()) {
      try {
        System.out.println("Gyro is calibrating!");
        Thread.sleep(200);
      } catch(InterruptedException exception) {
        Thread.currentThread().interrupt();
      }
      yawError = 0;
    }*/

    //System.out.println("Initial Heading Values::: " + gyro.getRotation2d());
  }

   /**
    * resets the odometer
    * @param pose
    */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    //gyro.reset();
    System.out.println("Heading before:: " + gyro.getRotation2d());
    odometry.resetPosition(pose, gyro.getRotation2d());
    System.out.println("Heading after:: " + gyro.getRotation2d());
  }

  public double getTurnRate() {
    return -gyro.getRate();
  }
  
  /**
   * Updates the odometer everytime it is called. Remember: a odometer needs Distance and rotation, not Rotation and velocity. :)
   */
  @Override
  public void periodic() {
    odometry.update(gyro.getRotation2d(),getLeftDistanceMeters(), getRightDistanceMeters()); 
      
     System.out.println("LEFT MOTOR DISTANCE:::::: " + getLeftDistanceMeters());
     System.out.println("RIGHT MOTOR DISTANCE:::::: " + getRightDistanceMeters());
    }  
}
