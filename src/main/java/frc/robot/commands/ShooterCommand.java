package frc.robot.commands;

import org.opencv.core.Mat;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;

public class ShooterCommand extends CommandBase {
	private Limelight limelight;
	private Shooter shooter;

	public ShooterCommand(Limelight limelight, Shooter shooter) {
		this.limelight = limelight;
		this.shooter = shooter;
		addRequirements(limelight, shooter);
	}

	// Called when the command is first scheduled.
	@Override
	public void initialize() {

	}

	// Called at 50hz while the command is scheduled.
	@Override
	public void execute() {
		final double horizontalAngleAbs = Math.abs(limelight.getHorizontalAngle());
		final double verticalAngleAbs = Math.abs(limelight.getHorizontalAngle());
		// Checks if aimed
		if (horizontalAngleAbs < Constants.SHOOTER_LIMELIGHT_ANGLE && verticalAngleAbs < Constants.SHOOTER_LIMELIGHT_ANGLE) {
			shooter.spool(Constants.SHOOTER_RPM);
		} else {
			shooter.stopMotor();
		}
	}

	// Called once when the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		shooter.stopMotor();
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
