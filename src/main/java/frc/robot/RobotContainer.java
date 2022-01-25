// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.commands.ClimbCommand;
import frc.robot.commands.IndexBall;
import frc.robot.commands.IntakeBall;
import frc.robot.commands.SwerveDrive;
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

  //Commands
  private IndexBall indexBall = new IndexBall(intake, storage);
  private ClimbCommand climbCommand = new ClimbCommand(climb);
  private SwerveDrive swerveDrive;

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
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    /*Sample auto to work off*/

    //create trajectory settings
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(Constants.Motor.SWERVE_MAX_SPEED,Constants.Motor.SWERVE_MAX_ACCELERATION).setKinematics(Constants.Motor.SWERVE_DRIVE_KINEMATICS);

    //generate trajectory
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0, 0, new Rotation2d(0)),
      List.of(
        new Translation2d(1, 0),
        new Translation2d(1, -1)),
      new Pose2d(2, -1, Rotation2d.fromDegrees(180)),
      trajectoryConfig);

      //pid controllers for tracking trajectory
      PIDController xController = new PIDController(Constants.PID.P_X_CONTROLLER, 0, 0);
      PIDController yController = new PIDController(Constants.PID.P_Y_CONTROLLER, 0, 0);
      ProfiledPIDController thetaController = new ProfiledPIDController(
              Constants.PID.P_THETA_CONTROLLER, 0, 0, Constants.Motor.THETA_CONTROL_CONSTRAINTS);
      thetaController.enableContinuousInput(-Math.PI, Math.PI);

      //construct command to follow trajectory
      SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        trajectory, 
        swerve::getPose, 
        Constants.Motor.SWERVE_DRIVE_KINEMATICS, 
        xController, 
        yController, 
        thetaController, 
        swerve::setModuleStates, 
        swerve);

    return new SequentialCommandGroup(
      new InstantCommand(() -> swerve.resetOdometry(trajectory.getInitialPose())),
      swerveControllerCommand,
      new InstantCommand(() -> swerve.stopModules())
    );
  }
}
