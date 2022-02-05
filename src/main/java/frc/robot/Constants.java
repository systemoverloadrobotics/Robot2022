// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.util.ConstantAxis;
import frc.robot.util.ConstantButton;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  //Distance to the Mid Bar
  public static final double CLIMBER_ENCODER_DISTANCE_MID = 12; 
  //Distance to the Low Bar
  public static final double CLIMBER_ENCODER_DISTANCE_LOW = 8;
  //setpoint for pulling the robot off the ground using random values
  public static final double RETRACTER_ENCODER_DISTANCE_MID = 12 *  -0.5
  public static final double RETRACTER_ENCODER_DISTANCE_LOW = 8 *  -0.5

  public static final class PID {
    public static final double P_CLIMB = 0.1; 
    public static final double I_CLIMB = 1e-4;
    public static final double D_CLIMB = 1; 
  }
	public static final class RobotDimensions {

    // Distance between wheels
    public static final double WIDTH = 28; //inches
    public static final double LENGTH = 28; //inches
  }
  
  public static final class Input {
    public static final ConstantAxis X_AXIS = new ConstantAxis(1, 1);
    public static final ConstantAxis Y_AXIS = new ConstantAxis(1, 2);
    public static final ConstantAxis ROTATION = new ConstantAxis(2, 1);
    public static final ConstantButton CLIMB_MID_BUTTON = new ConstantButton(1, 1);
    public static final ConstantButton CLIMB_LOW_BUTTON = new ConstantButton(1, 1);
    public static final ConstantButton RETRACT_MID_BUTTON = new ConstantButton(1, 1);
    public static final ConstantButton RETRACT_LOW_BUTTON = new ConstantButton(1, 1);
  }

  public static final class MotorSettings {
    public static final double MOTOR_VELOCITY = 0.5; 
    public static final int MOTOR_EXPIRATION = 4; 
  }

  public static final class Motor {
    public static final int SWERVE_FRONT_LEFT_POWER = 0;
    public static final int SWERVE_FRONT_LEFT_STEER = 1;

    public static final int SWERVE_FRONT_RIGHT_POWER = 2;
    public static final int SWERVE_FRONT_RIGHT_STEER = 3;

    public static final int SWERVE_BACK_LEFT_POWER = 4;
    public static final int SWERVE_BACK_LEFT_STEER = 5;

    public static final int SWERVE_BACK_RIGHT_POWER = 6;
    public static final int SWERVE_BACK_RIGHT_STEER = 7;

    public static final int EXAMPLE_SHOOTER_PORT = 2;
    public static final int EXAMPLE_INTAKE_CHANNEL = 3;

    public static final int LEFT_CLIMB_MOTOR = 0; //reset to actual later
    public static final int RIGHT_CLIMB_MOTOR = 1; //reset to actual later

  }

  public static final class Sensor {
    public static final int SWERVE_GYRO = 0;

    public static final int WHEEL_ENCODER_CHANNEL_A = 4;
    public static final int WHEEL_ENCODER_CHANNEL_B = 6;
  }

}
