// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.signals.S1StateValue;

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

  public static final class FuelConstants {
    // Motor controller IDs for Fuel Mechanism motors
    public static final int LEFT_INTAKE_LAUNCHER_MOTOR_ID = 50;
    public static final int RIGHT_INTAKE_LAUNCHER_MOTOR_ID = 51;
    public static final int INDEXER_MOTOR_ID = 3;

    // Current limit for fuel mechanism motors.
    public static final int INDEXER_MOTOR_CURRENT_LIMIT = 40;
    public static final int LAUNCHER_MOTOR_CURRENT_LIMIT = 40;

    // All values likely need to be tuned based on your robot
    public static final double INDEXER_INTAKING_PERCENT = -.6; 
    public static final double INDEXER_LAUNCHING_PERCENT = 0.6;
    public static final double INDEXER_SPIN_UP_PRE_LAUNCH_PERCENT = -0.5;

    public static final double INTAKE_INTAKING_PERCENT = 0.6;
    public static final double LAUNCHING_LAUNCHER_PERCENT = .85;
    public static final double ForcePull = 1;
    public static final double INTAKE_EJECT_PERCENT = -0.6;

    public static final double SPIN_UP_SECONDS = 0.75;
  }

  public static final class OperatorConstants {

    // Port constants for driver and operator controllers. These should match the
    // values in the Joystick tab of the Driver Station software
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;

    // This value is multiplied by the joystick value when rotating the robot to
    // help avoid turning too fast and beign difficult to control
    public static final double DRIVE_SCALING = 0.7;
    public static final double ROTATION_SCALING = 0.8;
  }

  public static final class VisionConstants {
    public static final String LIMELIGHT_NAME = "limelight-back";
    public static final int APRILTAG_PIPELINE = 0;

    public static final int[] VALID_TAG_IDS = {};

    public static final double ALIGN_STRAFE_KP = 1.6;
    public static final double ALIGN_ROTATION_KP = 0.08;

    public static final double ALIGN_MAX_STRAFE_METERS_PER_SECOND = 1.5;
    public static final double ALIGN_MAX_ROTATION_RADIANS_PER_SECOND = 1.5;

    public static final double ALIGN_LATERAL_TOLERANCE_METERS = 0.05;
    public static final double ALIGN_HEADING_TOLERANCE_DEGREES = 2.0;
    public static final double ALIGN_DESIRED_LATERAL_OFFSET_METERS = 0.0;

    public static final double MAX_VISION_ANGULAR_RATE_DEGREES_PER_SECOND = 540.0;
    public static final double MAX_VISION_TAG_AMBIGUITY = 0.7;
    public static final double MAX_VISION_AVG_TAG_DISTANCE_METERS = 4.5;

    private VisionConstants() {}
  }

  public static final class KrakenFuelConstants {

    // Can ID's MUST be set before trying it please :(

    public static final int KLeftIntakeLauncherID = 0;
    public static final int KRightIntakeLauncherID = 0;

    public static final int INDEXER_MOTOR_ID = 3;

    // The following values below are for testing reasons and aren't likely to be used at comp
    public static final double TESTSPEED = 0.1;
    public static final double TESTSPEEDBUTNEGATIVE = -0.1;

    // Current limit for fuel mechanism motors.
    public static final int INDEXER_MOTOR_CURRENT_LIMIT = 40;

    // All values likely need to be tuned based on your robot
    public static final double INDEXER_INTAKING_PERCENT = -.6; 
    public static final double INDEXER_LAUNCHING_PERCENT = 0.6;
    public static final double INDEXER_SPIN_UP_PRE_LAUNCH_PERCENT = -0.5;
    
  }
}
