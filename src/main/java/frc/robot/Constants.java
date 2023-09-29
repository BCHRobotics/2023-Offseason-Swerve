// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or5 share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.SerialPort;
import frc.robot.util.control.SparkMaxConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class CHASSIS {
    public static final int FRONT_LEFT_ID = 10;
    public static final int FRONT_RIGHT_ID = 11;
    public static final int BACK_LEFT_ID = 12;
    public static final int BACK_RIGHT_ID = 13;

    // Drivetrain restrictions
    public static final double DEFAULT_OUTPUT = 0.7;
    public static final double MAX_INTERVAL = 1 - DEFAULT_OUTPUT;
    public static final double MIN_INTERVAL = DEFAULT_OUTPUT - 0.2;
    public static final double RAMP_RATE = 0.15; // s
    public static final double TOLERANCE = 1; // in
    public static final boolean INVERTED = false;

    // Chassis dimensions needed
    public static final double WHEEL_DIAMETER = 6;
    public static final double GEAR_RATIO = 7.2;
    // public static final double TRACK_WIDTH = 19; // Measured [Inches]
    public static final double TRACK_WIDTH = 24; // Effective [Inches]

    // Chasis conversion factors TODO: Re-collect conversion data
    public static final double LEFT_POSITION_CONVERSION = 48 / 18.23804473876953; // inches / revs
    public static final double RIGHT_POSITION_CONVERSION = 48 / 18.14280891418457; // inches / revs
    public static final double POSITION_CONVERSION = (WHEEL_DIAMETER * Math.PI) / GEAR_RATIO;

    public static final double LEFT_VELOCITY_CONVERSION = LEFT_POSITION_CONVERSION / 60.0; // inches / s
    public static final double RIGHT_VELOCITY_CONVERSION = RIGHT_POSITION_CONVERSION / 60.0; // inches / s
    public static final double VELOCITY_CONVERSION = POSITION_CONVERSION / 60;

    // input diameter = Δd inches between center wheels ~~v~~ in inches / degree
    public static final double TURNING_CONVERSION = ((TRACK_WIDTH * Math.PI) / 360);

    // Drive PID Constants TODO: Re-tune Drivetrain PID
    public static final SparkMaxConstants LEFT_DRIVE_CONSTANTS = new SparkMaxConstants(
        0.00017, 0, 0.0025, 0, 0.00005, -1, 1, 0, 0, 4500, 1000, 0.05);
    public static final SparkMaxConstants RIGHT_DRIVE_CONSTANTS = new SparkMaxConstants(
        0.00017, 0, 0.0025, 0, 0.00005, -1, 1, 0, 0, 4500, 1000, 0.05);

    // Gyro constants
    public static final SerialPort.Port GYRO_PORT = SerialPort.Port.kMXP;
    public static final boolean GYRO_OUTPUT_INVERTED = false;
    public static final double GYRO_TOLERANCE = 0.8;

    // Gyro balance PID Constants TODO: Re-tune gyro
    public static final class BALANCE_CONSTANTS {
      public static final double kP = 0.006;
      public static final double kI = 0.001;
      public static final double kD = 0.0006;
      public static final double kFF = 0.002;
    }

    // Limelight PID constants
    public static final class SEEK_CONSTANTS {
      public static final double kP = 0.012;
      public static final double kI = 0;
      public static final double kD = 0.0002;
      public static final double kFF = 0.0132;
    }

    // Limelight PID constants
    public static final class TARGET_CONSTANTS {
      public static final double kP = 0.017;
      public static final double kI = 0;
      public static final double kD = 0.0002;
      public static final double kFF = 0.014;
    }

    // Gyro Turn PID constants
    public static final class ALIGN_CONSTANTS {
      public static final double kP = 0.0122;
      public static final double kI = 0;
      public static final double kD = 0.0002;
      public static final double kFF = 0.0132;
    }
  }
  
  public static final class PERIPHERALS {
    public static final int DRIVER_PORT = 0;
    public static final int OPERATOR_PORT = 1;
    public static final double CONTROLLER_DEADBAND = 0.1;
  }

  public static final class VISION {
    public enum TARGET_TYPE {
      APRILTAG, REFLECTIVE_TAPE, CONE, CUBE
    }

    public static final int CUBE_PIPELINE = 0;
    public static final int CONE_PIPELINE = 1;
    public static final int REFLECTIVE_PIPLINE = 3;
    public static final int NEURAL_NETWORK_PIPELINE = 4;
    public static final int APRILTAG_PIPELINE = 7;

    public static final double LIMELIGHT_ANGLE = 13.7; // degrees
    public static final double LIMELIGHT_HEIGHT = 43.5; // inches
    public static final double LIMELIGHT_TOLERANCE = 0.15; // degrees (x axis)
    public static final double LIMELIGHT_CHASSIS_OFFSET = 13; // inches

    public static final double MID_ARM_OFFSET = 42; // inches

    public static final double APRILTAG_HEIGHT = 18.125; // inches
    public static final double REFLECTIVE_TAPE_HEIGHT = 24.125; // inches
    public static final double CONE_HEIGHT = 43.75; // inches
    public static final double CUBE_HEIGHT = 4.5; // inches
  }

  public static final class MISC {
    public static final int CONE_LED_PORT = 0;
    public static final int CUBE_LED_PORT = 1;

    public static final double CUBE_DETECTION_CURRENT = 22;
    public static final double CONE_DETECTION_CURRENT = 29;

    public static final double BLINK_INTERVAL = 0.3; // seconds

    public static final double ENSURE_RANGE(double value, double min, double max) {
      return Math.min(Math.max(value, min), max);
    }

    public static final boolean IN_RANGE(double value, double min, double max) {
      return (value >= min) && (value <= max);
    }

    public static final boolean WITHIN_TOLERANCE(double value, double tolerance) {
      return (value >= -tolerance) && (value <= tolerance);
    }

    public static final boolean WITHIN_TOLERANCE(double input, double setpoint, double tolerance) {
      return (input >= (setpoint - tolerance)) && (input <= (setpoint + tolerance));
    }
  }
}
