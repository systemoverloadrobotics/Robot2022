package frc.robot.commands.storage;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Storage;
import frc.robot.subsystems.Storage.ToggleState;

public class ClearStorage extends CommandBase {

	private Storage storage;
	private Intake intake;

	public ClearStorage(Storage storage, Intake intake) {
		this.storage = storage;
		this.intake = intake;
		addRequirements(storage, intake);
	}

	// Called when the command is first scheduled.
	@Override
	public void initialize() {
		intake.actuate();
	}

	// Called at 50hz while the command is scheduled.
	@Override
	public void execute() {
		storage.reverseFeeder();
		storage.toggleBelt(ToggleState.REVERSE);
		intake.reverseIntake();
	}

	// Called once when the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		storage.stop();
		intake.intakeBall(0);
	}

	@Override
	public boolean isFinished() {
		return false;
	}

}
