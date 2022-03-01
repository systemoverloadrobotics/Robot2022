package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

public class ShooterCommand extends CommandBase {
	private Limelight limelight;
	private Shooter shooter;
	private Swerve swerve;

	public ShooterCommand(Limelight limelight, Shooter shooter, Swerve swerve) {
		this.limelight = limelight;
		this.shooter = shooter;
		this.swerve = swerve;
		addRequirements(limelight, shooter);
	}

	// Called when the command is first scheduled.
	@Override
	public void initialize() {

	}

	// Called at 50hz while the command is scheduled.
	@Override
	public void execute() {
		if ( new AimCommand(limelight, swerve).isFinished()) {
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
