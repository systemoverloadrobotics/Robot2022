// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.ClimbCommand;
import frc.robot.subsystems.Climb;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private Climb climb = new Climb();
  private Command extendLowClimbCommand = new CommandBase() {
    {
      addSubsystems(climb);
    }

    public void execute() {
      if (climb.getEncoderValue() != Constants.CLIMBER_ENCODER_DISTANCE_LOW) {
				climb.setSetpoint(Constants.CLIMBER_ENCODER_DISTANCE_LOW);
			} else {
				climb.stop();
			}
    }

    public void end(boolean interrupted) {
      climb.stop()
    }
  }

  private Command extendMidClimbCommand = new CommandBase() {
    {
      addSubsystems(climb);
    }

    public void execute() {
      if (climb.getEncoderValue() != Constants.CLIMBER_ENCODER_DISTANCE_MID) {
				climb.setSetpoint(Constants.CLIMBER_ENCODER_DISTANCE_MID);
			} else {
				climb.stop();
			}
    }

    public void end(boolean interrupted) {
      climb.stop()
    }
  }

  private Command retractLowCommand = new CommandBase() {
    {
      addSubsystems(climb);
    }

    public void execute() {
      if (climb.getEncoderValue() != Constants.RETRACTER_ENCODER_DISTANCE_LOW) {
				climb.setSetpoint(Constants.RETRACTER_ENCODER_DISTANCE_LOW);
			} else {
				climb.stop();
			}
    }
  }
  private Command retractMidCommand = new CommandBase() {
    {
      addSubsystems(climb);
    }

    public void execute() {
      if (climb.getEncoderValue() != Constants.RETRACTER_ENCODER_DISTANCE_MID) {
				climb.setSetpoint(Constants.RETRACTER_ENCODER_DISTANCE_MID);
			} else {
				climb.stop();
			}
    }
  }

  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    Constants.Input.CLIMB_MID_BUTTON.get().whenPressed(extendMidClimbCommand); 
    Constants.Input.CLIMB_LOW_BUTTON.get().whenPressed(extendLowClimbCommand);
    Constants.Input.RETRACT_MID_BUTTON.get().whenPressed(retractMidCommand); 
    Constants.Input.RETRACT_LOW_BUTTON.get().whenPressed(retractLowCommand);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }
}
