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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.commands.ShooterCommand;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Storage;
import frc.robot.subsystems.Swerve;

public class AutoPaths{
  /* Sample Auto for Swerve*/
  private static Swerve swerve;

  public AutoPaths(Swerve swerve){
    this.swerve = swerve;
  }

  public static Command exampleAuto(Storage storage, Shooter shooter) {
    //create trajectory settings

    //generate trajectory
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0, 0, new Rotation2d()),
      List.of(
        new Translation2d(0, 0),
        new Translation2d(0, 0)),
      new Pose2d(-1, 0, new Rotation2d()),
      Constants.Motor.TRAJECTORY_CONFIG);

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
      new ShooterCommand(storage, shooter),
      swerveControllerCommand,
      new InstantCommand(() -> swerve.stopModules()));
  }
  
}
