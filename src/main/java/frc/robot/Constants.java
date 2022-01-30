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

  public static final double CLIMBER_ENCODER_DISTANCE = 12; 
  

  public static final class PID {
    public static final double P_CLIMB = 0.1; 
    public static final double I_CLIMB = 1e-4;
    public static final double D_CLIMB = 1; 

    public static final double P_FEEDER = 0.7;
  }
	public static final class RobotDimensions {

    // Distance between wheels
    public static final double WIDTH = 28; //inches
    public static final double LENGTH = 28; //inches

    //Feeder 
    public static final double FEEDER_DIAMETER = 2.0; //inches
    public static final double FEEDER_ENCODER_DISTANCE_PER_PULSE = (1/8192) * FEEDER_DIAMETER * Math.PI; 
    public static final double FEEDER_OFFSET_DISTANCE = 2.0; //inches
  }
  
  public static final class Input {

    //Axis
    public static final ConstantAxis X_AXIS = new ConstantAxis(1, 1);
    public static final ConstantAxis Y_AXIS = new ConstantAxis(1, 2);
    public static final ConstantAxis ROTATION = new ConstantAxis(2, 1);

    //Buttons
    public static final ConstantButton CLIMB_BUTTON = new ConstantButton(1, 1); 
    public static final ConstantButton INTAKE_BUTTON = new ConstantButton(1, 5); 
    public static final ConstantButton STORAGE_TOGGLE = new ConstantButton(1, 0);
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
    
    public static final int STORAGE_MOVEMENT_BELT = 8;
    public static final int STORAGE_FEEDER = 0;
  
    public static final int EXAMPLE_SHOOTER_PORT = 2;
    public static final int EXAMPLE_INTAKE_CHANNEL = 3;

    public static final int INTAKE = 5;
    public static final double INTAKE_SPEED = 0.5;

    public static final int LEFT_CLIMB_MOTOR = 0; //reset to actual later
    public static final int RIGHT_CLIMB_MOTOR = 1; //reset to actual later

  }

  public static final class Sensor {
    //Swerve
    public static final int SWERVE_GYRO = 0;

    //Climb
    public static final int CLIMB_INTAKE_SENSOR = 0;
    public static final int CLIMB_STORAGE_SENSOR = 1;
    public static final int CLIMB_SHOOTER_SENSOR = 2;

    //Shooter
    public static final int FEEDER_ENCODER_CHANNEL_A = 3;
    public static final int FEEDER_ENCODER_CHANNEL_B = 4;
  }

}
