package frc.robot.commands.auto;

import java.util.List;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Storage;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Storage.ToggleState;

public class AutoPaths {
  /* Sample Auto for Swerve */
  private static Swerve swerve;
  private static Intake intake;
  private static Storage storage;

  public AutoPaths(Swerve swerve, Intake intake, Storage storage) {
    this.swerve = swerve;
    this.intake = intake;
    this.storage = storage;
  }

  public static Command twoBallAuto() {
        //move backwards


    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(Constants.Motor.SWERVE_MAX_SPEED, Constants.Motor.SWERVE_MAX_ACCELERATION);

    Trajectory trajectory =
        TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(0)),
            List.of(new Translation2d(-1, 0), new Translation2d(0, 7)),
            new Pose2d(0, 7, Rotation2d.fromDegrees(0)), trajectoryConfig);
    
    PIDController xController = new PIDController(Constants.PID.P_X_CONTROLLER, 0, 0);
    PIDController yController = new PIDController(Constants.PID.P_Y_CONTROLLER, 0, 0);
    ProfiledPIDController thetaController = new ProfiledPIDController(
        Constants.PID.P_THETA_CONTROLLER, 0, 0, Constants.Motor.THETA_CONTROL_CONSTRAINTS);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand initialMovementCommand = new SwerveControllerCommand(trajectory,
    swerve::getPose, Constants.Motor.SWERVE_DRIVE_KINEMATICS, xController, yController,
    thetaController, swerve::setModuleStates, swerve);

    SwerveControllerCommand moveToShoot = new SwerveControllerCommand(trajectory,
    swerve::getPose, Constants.Motor.SWERVE_DRIVE_KINEMATICS, xController, yController,
    thetaController, swerve::setModuleStates, swerve);

    //roll intake 
    //run storage
    //move forward 
    //shoot after you are close enough  

    Command intakeCommand = new CommandBase(){
      {
        addRequirements(intake);
      }
      @Override
      public void execute() {
        intake.intakeBall(Constants.Motor.INTAKE_SPEED);
      }
    };

    
    Command storageCommand = new CommandBase() {
      {
        addRequirements(storage);
      }
      @Override
      public void execute() {
        storage.toggleBelt(ToggleState.ON);
      }
    };
    
    return new SequentialCommandGroup(new InstantCommand(() -> swerve.resetOdometry(trajectory.getInitialPose())), new ParallelCommandGroup(initialMovementCommand, intakeCommand, storageCommand)); 
      
    
  }

  public static Command exampleAuto() {
    // create trajectory settings
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(Constants.Motor.SWERVE_MAX_SPEED,
        Constants.Motor.SWERVE_MAX_ACCELERATION)
            .setKinematics(Constants.Motor.SWERVE_DRIVE_KINEMATICS);

    // generate trajectory
    Trajectory trajectory =
        TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(0)),
            List.of(new Translation2d(1, 0), new Translation2d(1, -1)),
            new Pose2d(2, -1, Rotation2d.fromDegrees(180)), trajectoryConfig);

    // pid controllers for tracking trajectory
    PIDController xController = new PIDController(Constants.PID.P_X_CONTROLLER, 0, 0);
    PIDController yController = new PIDController(Constants.PID.P_Y_CONTROLLER, 0, 0);
    ProfiledPIDController thetaController = new ProfiledPIDController(
        Constants.PID.P_THETA_CONTROLLER, 0, 0, Constants.Motor.THETA_CONTROL_CONSTRAINTS);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // construct command to follow trajectory
    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(trajectory,
        swerve::getPose, Constants.Motor.SWERVE_DRIVE_KINEMATICS, xController, yController,
        thetaController, swerve::setModuleStates, swerve);

    return new SequentialCommandGroup(
        new InstantCommand(() -> swerve.resetOdometry(trajectory.getInitialPose())),
        swerveControllerCommand, new InstantCommand(() -> swerve.stopModules()));


  }

}
