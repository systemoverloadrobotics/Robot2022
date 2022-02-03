package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class EjectBall extends CommandBase {

	private Intake intake;

	public EjectBall(Intake intake) {
		this.intake = intake;
		addRequirements(intake);
	}

	// Called when the command is first scheduled.
	@Override
	public void initialize() {}

	// Called at 50hz while the command is scheduled.
	@Override
	public void execute() {
		intake.reverseIntake();
	}

	// Called once when the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		intake.intakeBall(0);
	}

	@Override
	public boolean isFinished() {
		return false;
	}

}
