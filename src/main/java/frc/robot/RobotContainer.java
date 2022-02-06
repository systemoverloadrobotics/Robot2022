// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.ClimbCommand;
import frc.robot.commands.IndexBall;
import frc.robot.commands.SwerveDrive;
import frc.robot.commands.auto.AutoPaths;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Storage;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Storage.ToggleState;

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
  private Shooter shooter = new Shooter();

  //Commands
  private IndexBall indexBall = new IndexBall(intake, storage);
  private ClimbCommand climbCommand = new ClimbCommand(climb);
  private SwerveDrive swerveDrive;
  private Command spoolCommand = new CommandBase() {
    {
      addRequirements(shooter);
    }

    public void execute() {
      shooter.spool(Constants.SHOOTER_RPM);
    };
  };
  private Command shootCommand = new CommandBase() {
    {
      addRequirements(shooter);
    }

    public void execute() {
      storage.toggleBelt(ToggleState.ON);
      if (shooter.shooterMotorRPM() >= Constants.SHOOTER_RPM) {
        shooter.spool(Constants.SHOOTER_RPM);
      }
    };
  };


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    swerveDrive = new SwerveDrive(swerve, Constants.Input.X_AXIS.get(), Constants.Input.Y_AXIS.get(), Constants.Input.ROTATION.get());
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
    Constants.Input.INTAKE_BUTTON.get().whileHeld(indexBall);
    Constants.Input.SHOOTER_SPOOL.get().whileHeld(spoolCommand);
    Constants.Input.SHOOTER_SHOOT.get().whileHeld(shootCommand);
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
