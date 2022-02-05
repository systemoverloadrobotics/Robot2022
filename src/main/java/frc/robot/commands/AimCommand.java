package frc.robot.commands;

import java.util.function.Supplier;
import org.opencv.core.Mat;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

public class AimCommand extends CommandBase {
	private Limelight limelight;
	private Swerve swerve;

	public AimCommand(Limelight limelight, Swerve swerve) {
		this.limelight = limelight;
		this.swerve = swerve;
		addRequirements(limelight, swerve);
	}

	// Called when the command is first scheduled.
	@Override
	public void initialize() {

	}

	// Called at 50hz while the command is scheduled.
	@Override
	 public void execute() {
		 double tx = limelight.getHorizontalAngle();
		 double ty = limelight.getVerticalAngle();

		 if (Math.abs(tx) > 0.5 || Math.abs(ty) > 0.5) {
			 swerve.setModuleStates(Constants.Motor.SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, Math.PI / 2D, swerve.getRotation2d())));
		 }
	 }

	// Called once when the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		// swerve.drive(0, 0, 0);
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
