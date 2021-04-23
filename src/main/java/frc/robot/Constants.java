// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class PhysicalRobotConstants {
        public static final double kTrackWidthMeters = 0.552976;
        //change these values for yith
        public static final double kS = 0.579; //Volts
        public static final double kV = 2.37; //VoltsPerMeter
        public static final double kA = 0.21; //VoltsPerMeterSquared
        public static final double feetPerTick = 1.2207031E-4; //feet conversion from ticks(one cycle on talon)
        public static final double metersPerTick = (2048/4.67) * (1000/Math.PI * 6 * 2.54);
        public static final double kMaxVoltage = 8; //max is 12V
        public static final double tickSpeedInMetersPerSec = 1861.2;
        public static final int kFalconCPR = 2048;
        public static final double kGearRatio = 4.67;
        public static final double kWheelDiameterMeters = 0.1524;
        

        

       

        
      
    }

    public static final class DriveTalonIDs {
        public static final int leftFrontID = 3;
        public static final int leftBackID = 2;
        public static final int rightFrontID = 0;
        public static final int rightBackID = 1;
    }

    public static final class DriveConstants {
        //change these values for yith
        public static final double kP = 0.00048; //2.21 is for WPILib PID integration, use 0.00048 for TalonFX PID integration
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double maxV = 3; //MetersPerSecond
        public static final double maxA = 1.5; //MetersPerSecondSquared
        public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(PhysicalRobotConstants.kTrackWidthMeters);
        

    }
}

