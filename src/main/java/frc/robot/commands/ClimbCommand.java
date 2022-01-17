package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climb;

public class ClimbCommand extends CommandBase {

	private Climb climb;
	private boolean isClimbing;

	public ClimbCommand(Climb climb) {
		this.climb = climb;
		addRequirements(climb);
	}

	// Called when the command is first scheduled.
	@Override
	public void initialize() {
		isClimbing = false;
	}

	// Called at 50hz while the command is scheduled.
	@Override
	public void execute() {
		if (this.isClimbing) {
			isClimbing = true;
			climb.climbUp();
		} else {
			climb.recall();
		}
	}

	// Called once when the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {}

	@Override
	public boolean isFinished() {
		return false;
	}

}
