// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.IndexBall;
import frc.robot.commands.SwerveDrive;
import frc.robot.commands.auto.AutoPaths;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Storage;
import frc.robot.subsystems.Swerve;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...


    //Subsystems
    private Climb climb = new Climb(); 
    private Intake intake = new Intake();
    private Storage storage = new Storage();
    private Swerve swerve = new Swerve();
  
    private IndexBall indexBall = new IndexBall(intake, storage);
    private SwerveDrive swerveDrive;
  
    // Commands
    // private IndexBall indexBall = new IndexBall(intake, storage);
    // private ClimbCommand climbCommand = new ClimbCommand(climb);
    private Command extendLowClimbCommand = new CommandBase() {
      {
        addRequirements(climb);
      }

      public void execute() {
        if (climb.getEncoderValue() != Constants.CLIMBER_ENCODER_DISTANCE_LOW) {
          climb.setSetpoint(Constants.CLIMBER_ENCODER_DISTANCE_LOW);
        } else {
          climb.stop();
        }
      }

      public void end(boolean interrupted) {
        climb.stop();
      }
    };

    private Command extendMidClimbCommand = new CommandBase() {
      {
        addRequirements(climb);
      }

      public void execute() {
        if (climb.getEncoderValue() != Constants.CLIMBER_ENCODER_DISTANCE_MID) {
          climb.setSetpoint(Constants.CLIMBER_ENCODER_DISTANCE_MID);
        } else {
          climb.stop();
        }
      }

      public void end(boolean interrupted) {
        climb.stop();
      }
    };

    private Command retractLowCommand = new CommandBase() {
      {
        addRequirements(climb);
      }

      public void execute() {
        if (climb.getEncoderValue() != Constants.RETRACTER_ENCODER_DISTANCE_LOW) {
          climb.setSetpoint(Constants.RETRACTER_ENCODER_DISTANCE_LOW);
        } else {
          climb.stop();
        }
      }
    };

    private Command retractMidCommand = new CommandBase() {
      {
        addRequirements(climb);
      }

      public void execute() {
        if (climb.getEncoderValue() != Constants.RETRACTER_ENCODER_DISTANCE_MID) {
          climb.setSetpoint(Constants.RETRACTER_ENCODER_DISTANCE_MID);
        } else {
          climb.stop();
        }
      }
    };

    private Command climbExtendCommand = new CommandBase() {
      {
        addRequirements(climb);
      }

      public void execute() {
        climb.overrideUp();
      }

      public void end(boolean interrupted) {
        climb.stop();
      };
    };

    private Command climbRetractCommand = new CommandBase() {
      {
        addRequirements(climb);
      }

      public void execute() {
        climb.overrideDown();
      }

      public void end(boolean interrupted) {
        climb.stop();
      };
    };

  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    Constants.Input.CLIMB_MID_BUTTON.get().whenPressed(extendMidClimbCommand); 
    Constants.Input.CLIMB_LOW_BUTTON.get().whenPressed(extendLowClimbCommand);
    Constants.Input.RETRACT_MID_BUTTON.get().whenPressed(retractMidCommand); 
    Constants.Input.RETRACT_LOW_BUTTON.get().whenPressed(retractLowCommand);
    Constants.Input.OVERRIDE_UP_CLIMB.get().whenPressed(climbExtendCommand);
    Constants.Input.OVERRIDE_DOWN_CLIMB.get().whenPressed(climbRetractCommand);
    Constants.Input.INTAKE_BUTTON.get().whileHeld(indexBall);
    swerve.setDefaultCommand(new SwerveDrive(swerve, Constants.Input.X_AXIS.get(),
        Constants.Input.Y_AXIS.get(), Constants.Input.ROTATION.get()));

    // Constants.Input.CLIMB_BUTTON.get().whenPressed(climbCommand);
    // Constants.Input.INTAKE_BUTTON.get().whileHeld(indexBall);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    return new AutoPaths(swerve).exampleAuto();
  }
}
