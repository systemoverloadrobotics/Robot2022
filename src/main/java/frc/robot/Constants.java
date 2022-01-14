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

	public static final class RobotDimensions {

    //swerve drive dimensions
    public static final double WIDTH = 30; //inches
    public static final double LENGTH = 30; //inches
  }
  
  public static final class Input {
    public static final ConstantButton EXAMPLE_BUTTON = new ConstantButton(1, 4);
    public static final ConstantAxis X_AXIS = new ConstantAxis(1, 1);
    public static final ConstantAxis Y_AXIS = new ConstantAxis(1, 2);
    public static final ConstantAxis ROTATION = new ConstantAxis(2, 1);
  }

  public static final class Motor {
    public static final int EXAMPLE_SHOOTER_PORT = 2;
    public static final int EXAMPLE_INTAKE_CHANNEL = 3;
  }

  public static final class Sensor {
    public static final int WHEEL_ENCODER_CHANNEL_A = 4;
    public static final int WHEEL_ENCODER_CHANNEL_B = 6;
  }

}
