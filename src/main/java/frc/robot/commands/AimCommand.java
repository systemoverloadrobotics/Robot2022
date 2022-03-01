package frc.robot.commands;

import java.util.List;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;

public class AimCommand extends CommandBase {
	private Limelight limelight;
	private Swerve swerve;
	private ProfiledPIDController thetaController;
	private SwerveControllerCommand controllerCommand;

	public AimCommand(Limelight limelight, Swerve swerve) {
		this.limelight = limelight;
		this.swerve = swerve;
		this.thetaController = new ProfiledPIDController(Constants.PID.P_THETA_CONTROLLER, 0, 0, Constants.Motor.THETA_CONTROL_CONSTRAINTS);
		thetaController.enableContinuousInput(-Math.PI, Math.PI);
		addRequirements(limelight, swerve);
	}

	// Called when the command is first scheduled.
	@Override
	public void initialize() {

	}

	// Called at 50hz while the command is scheduled.
	@Override
	public void execute() {
		if(limelight.hasTarget()){
			double tx = limelight.getHorizontalAngle();
			Trajectory trajectoryError = TrajectoryGenerator.generateTrajectory(
				new Pose2d(0,0, new Rotation2d(0)),
				List.of(),
				new Pose2d(0,0, Rotation2d.fromDegrees(tx)),
				Constants.Motor.TRAJECTORY_CONFIG);
			controllerCommand = new SwerveControllerCommand(
				trajectoryError, 
				swerve::getPose, 
				Constants.Motor.SWERVE_DRIVE_KINEMATICS, 
				null, 
				null, 
				thetaController,
				swerve::setModuleStates,
				swerve);
			controllerCommand.execute();
		} else {
			swerve.setModuleStates(
				Constants.Motor.SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(
					ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 2, swerve.getRotation2d())));
		}
		swerve.stopModules();	
	}

	// Called once when the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
	}

	@Override
	public boolean isFinished() {
		return controllerCommand.isFinished();
	}	
}
