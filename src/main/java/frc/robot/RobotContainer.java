// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ActuateIntake;
import frc.robot.commands.IntakeBall;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.Spool;
import frc.robot.commands.SwerveDrive;
import frc.robot.commands.TestCommand;
import frc.robot.commands.auto.AutoPaths;
import frc.robot.commands.storage.IndexBall;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
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

  // Subsystems
  private Intake intake = new Intake();
  private Storage storage = new Storage();
  private Swerve swerve = new Swerve();
  private Shooter shooter = new Shooter();
  private GenericHID rightMaster = new GenericHID(0);
  private GenericHID leftMaster = new GenericHID(1);
  private XboxController joy = new XboxController(2);
  // Commands
  private IndexBall indexBall = new IndexBall(storage, intake);
  // private ClimbCommand climbCommand = new ClimbCommand(climb);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   *    * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    swerve.setDefaultCommand(new SwerveDrive(swerve, () -> rightMaster.getRawAxis(0),
        () -> rightMaster.getRawAxis(1), () -> leftMaster.getRawAxis(0)));
    JoystickButton aButton = new JoystickButton(leftMaster, 2);
    aButton.whenPressed(new InstantCommand(() -> swerve.resetHeading()));
    JoystickButton bBUtton = new JoystickButton(joy, 2);
    bBUtton.whenHeld(new IntakeBall(intake,storage));
    JoystickButton yButton = new JoystickButton(joy, 4);
    yButton.whenHeld(new TestCommand(storage));
    JoystickButton lbButton = new JoystickButton(joy, 5);
    lbButton.whenHeld(new Spool(shooter)); 
    JoystickButton rbButton = new JoystickButton(joy, 6);
    rbButton.whenHeld(new ShooterCommand(storage, shooter));

    //JoystickButton xButton = new JoystickButton(joy, 3);
    //xButton.whenPressed(new ActuateIntake(intake));

    // Constants.Input.CLIMB_BUTTON.get().whenPressed(climbCommand);
    // Constants.Input.INTAKE_BUTTON.get().whileHeld(indexBall);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return AutoPaths.exampleAuto(storage, shooter);
  }
}
