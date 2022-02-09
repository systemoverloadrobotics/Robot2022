package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Storage;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Storage.BallColor;
import frc.robot.subsystems.Storage.ToggleState;

public class AutoCamera extends CommandBase {

	private Limelight limelight;
	private Swerve swerve;
	private Storage storage;
	private Intake intake;
	private Shooter shooter;

	public AutoCamera(Limelight limelight, Swerve swerve, Storage storage, Intake intake, Shooter shooter) {
		this.limelight = limelight;
		this.swerve = swerve;
		this.storage = storage;
		this.intake = intake;
		this.shooter = shooter;
		addRequirements(limelight, swerve, storage, intake);
	}

	// Called when the command is first scheduled.
	@Override
	public void initialize() {}

	// Called at 50hz while the command is scheduled.
	@Override
	public void execute() {
		// Look for a ball
		if (!limelight.targetExists()) {
			swerve.setModuleStates(0, 0, Math.PI / 2);
			intake.intakeBall(0);
			storage.toggleBelt(ToggleState.OFF);
		}
		// Aim robot towards ball
		else if (Math.abs(limelight.getHorizontalAngle()) > 0.1) {
			swerve.setModuleStates(0, 0, Math.toRadians(limelight.getHorizontalAngle()));
			intake.intakeBall(0);
			storage.toggleBelt(ToggleState.OFF);
		}
		// Turn on Intake if there is no ball
		else if (storage.getColor() == BallColor.NONE) {
			intake.intakeBall(Constants.Motor.INTAKE_SPEED);
			swerve.setModuleStates(1, 0, 0);
		}
		else {
			intake.intakeBall(0);
		}
		
		// Shooting
		double requiredVelocity = Constants.BALL_VELOCITY_CONVERSION_TO_MOTOR_SPEED.apply(limelight.getVelocity());
		if (shooter.shooterMotorRPM() >= requiredVelocity) {
			storage.toggleBelt(ToggleState.ON);
		}
		else {
			storage.toggleBelt(ToggleState.OFF);
		}
		shooter.spool(requiredVelocity);
	}

	// Called once when the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		intake.intakeBall(0);
		storage.toggleBelt(ToggleState.OFF);
	}

	@Override
	public boolean isFinished() {
		return false;
	}

}