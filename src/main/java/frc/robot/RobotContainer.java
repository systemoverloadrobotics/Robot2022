// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.ClimbCommand;
import frc.robot.commands.EjectBall;
import frc.robot.commands.IndexBall;
import frc.robot.commands.IntakeBall;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Storage;

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

  //Commands
  private IndexBall indexBall = new IndexBall(intake, storage);
  private EjectBall ejectBall = new EjectBall(intake);
  private ClimbCommand climbCommand = new ClimbCommand(climb);

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
    Constants.Input.CLIMB_BUTTON.get().whenPressed(climbCommand); 
    Constants.Input.INTAKE_RUN.get().whileHeld(indexBall);
    Constants.Input.INTAKE_EJECT_BALL.get().whileHeld(ejectBall);
    Constants.Input.COMMAND_EMERGENCY_STOP.get().whenPressed(new CommandBase() {
      @Override
      public void execute() {
        CommandScheduler.getInstance().cancelAll();
      }
    });
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
