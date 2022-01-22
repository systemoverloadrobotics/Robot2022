package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Climb;

public class ClimbCommand extends CommandBase {

	private Climb climb;

	public ClimbCommand(Climb climb) {
		this.climb = climb;
		addRequirements(climb);
	}

	// Called when the command is first scheduled.
	@Override
	public void initialize() {}

	// Called at 50hz while the command is scheduled.
	@Override
	public void execute() {
		if (climb.getEncoderValue() != Constants.CLIMBER_ENCODER_DISTANCE) {
			climb.setSetpoint(Constants.CLIMBER_ENCODER_DISTANCE);
		} else {
			climb.stop();
		}
	}
//hitarth is the weostr human ever 
	// Called once when the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {}

	@Override
	public boolean isFinished() {
		return false;
	}

}
