// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.util.ConstantAxis;
import frc.robot.util.ConstantButton;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final boolean IS_REAL = RobotBase.isReal();
  public static final double CLIMBER_ENCODER_DISTANCE = 12;


  public static final class PID {
    // Climb
    public static final double P_CLIMB = 0.1;
    public static final double I_CLIMB = 1e-4;
    public static final double D_CLIMB = 1;

    public static final double P_FEEDER = 0.7;
    public static final double I_FEEDER = 0;
    public static final double D_FEEDER = 0;
    // Swerve
    public static final double P_SWERVE_STEER = 0.05;
    public static final double I_SWERVE_STEER = 0.025;
    public static final double D_SWERVE_STEER = 0.5;

    public static final double P_SWERVE_POWER = 0.03;
    public static final double I_SWERVE_POWER = 0.015;
    public static final double D_SWERVE_POWER = 0.3;

    // Linear drive feed forward
    public static final SimpleMotorFeedforward DRIVE_FF = IS_REAL ? new SimpleMotorFeedforward( // real
        0.6, // Voltage to break static friction
        2.5, // Volts per meter per second
        1 // Volts per meter per second squared
    )
        : new SimpleMotorFeedforward( // sim
            0, // Voltage to break static friction -- we do not use kS with this method of
               // simulation
            2.5, // Volts per meter per second
            0.4 // Volts per meter per second squared -- lower kA will give snappier control
        );
    // Steer feed forward
    public static final SimpleMotorFeedforward STEER_FF = IS_REAL ? new SimpleMotorFeedforward( // real
        0, // Voltage to break static friction
        0.15, // Volts per radian per second
        0.04 // Volts per radian per second squared
    )
        : new SimpleMotorFeedforward( // sim
            0, // Voltage to break static friction
            0.15, // Volts per radian per second
            0.002 // Volts per radian per second squared
        );

    // Controller
    public static final double P_X_CONTROLLER = 1.5;
    public static final double P_Y_CONTROLLER = 1.5;
    public static final double P_THETA_CONTROLLER = 3;
  }
  public static final class RobotDimensions {

    // Feeder
    public static final double FEEDER_DIAMETER = 2.0; // inches
    public static final double FEEDER_ENCODER_DISTANCE_PER_PULSE =
        (1 / 8192) * FEEDER_DIAMETER * Math.PI;
    public static final double FEEDER_OFFSET_DISTANCE = 2.0; // inches
    public static final double WIDTH = Units.inchesToMeters(28); // inches
    public static final double LENGTH = Units.inchesToMeters(28); // inches

    public static final double WHEEL_CIRCUMFRENCE = Units.inchesToMeters(3.5) * Math.PI;
  }

  public static final class Input {

    // Axis
    public static final ConstantAxis X_AXIS = new ConstantAxis(1, 1);
    public static final ConstantAxis Y_AXIS = new ConstantAxis(1, 2);
    public static final ConstantAxis ROTATION = new ConstantAxis(1, 3);

    // Buttons
    public static final ConstantButton CLIMB_BUTTON = new ConstantButton(1, 1);
    public static final ConstantButton INTAKE_BUTTON = new ConstantButton(1, 5);
    public static final ConstantButton REVERSE_INTAKE_BUTTON = new ConstantButton(1, 6);
    public static final ConstantButton CLEAR_STORAGE = new ConstantButton(1, 0);
  }

  public static final class MotorSettings {
    public static final double MOTOR_VELOCITY = 0.5;
    public static final int MOTOR_EXPIRATION = 4;
  }

  public static final class Motor {

    // Swerve
    public static final int SWERVE_FRONT_LEFT_POWER = 0;
    public static final int SWERVE_FRONT_LEFT_STEER = 1;

    public static final int SWERVE_FRONT_RIGHT_POWER = 2;
    public static final int SWERVE_FRONT_RIGHT_STEER = 3;

    public static final int SWERVE_BACK_LEFT_POWER = 4;
    public static final int SWERVE_BACK_LEFT_STEER = 5;

    public static final int SWERVE_BACK_RIGHT_POWER = 6;
    public static final int SWERVE_BACK_RIGHT_STEER = 7;

    public static final SwerveDriveKinematics SWERVE_DRIVE_KINEMATICS = new SwerveDriveKinematics(
        new Translation2d(RobotDimensions.LENGTH / 2, -RobotDimensions.WIDTH / 2),
        new Translation2d(RobotDimensions.LENGTH / 2, RobotDimensions.WIDTH / 2),
        new Translation2d(-RobotDimensions.LENGTH / 2, -RobotDimensions.WIDTH / 2),
        new Translation2d(-RobotDimensions.LENGTH, RobotDimensions.WIDTH / 2));

    public static final double SWERVE_POWER_GEAR_RATIO = 6.55;

    public static final double SWERVE_MAX_SPEED = 5.18; // m/s
    public static final double SWERVE_MAX_ACCELERATION = 3; // m/s^2
    public static final double SWERVE_ROTATION_MAX_SPEED = 2 * 2 * Math.PI; // rad/s
    public static final double SWERVE_ROTATION_MAX_ACCELERATION = Math.PI / 4; // rad/s^2
    public static final double SWERVE_NOMINAL_OUTPUT_PERCENT = 0.05;

    public static final double SWERVE_DEADBAND = 0.05;

    public static final TrapezoidProfile.Constraints THETA_CONTROL_CONSTRAINTS =
        new TrapezoidProfile.Constraints(SWERVE_ROTATION_MAX_SPEED,
            SWERVE_ROTATION_MAX_ACCELERATION);

    // Storage
    public static final int STORAGE_MOVEMENT_BELT = 8;
    public static final int STORAGE_FEEDER = 0;
    public static final double STORAGE_ON = 0.5;
    public static final double STORAGE_REVERSE = -0.5;
    public static final double STORAGE_FEEDER_ON = 0.5;
    public static final double STORAGE_FEEDER_REVERSE = -0.5;

    // Intake
    public static final int INTAKE = 5;
    public static final double INTAKE_SPEED = 0.5;
    public static final double INTAKE_REVERSE = -0.5;

    // Climb
    public static final int LEFT_CLIMB_MOTOR = 0; // reset to actual later
    public static final int RIGHT_CLIMB_MOTOR = 1; // reset to actual later

  }

  public static final class Sensor {
    // Swerve
    public static final int SWERVE_GYRO = 0;

    // Climb

    public static final int PROXIMITY_INTAKE_SENSOR = 1;
    public static final int PROXIMITY_STORAGE_SENSOR = 1;
    public static final int PROXIMITY_SHOOTER_SENSOR = 2;

    // Shooter
    public static final int FEEDER_ENCODER_CHANNEL_A = 3;
    public static final int FEEDER_ENCODER_CHANNEL_B = 4;
  }

}
