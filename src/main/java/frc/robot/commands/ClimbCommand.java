package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Climb;

public class ClimbCommand extends CommandBase {

	private Climb climb;
	private Bar bar;
	enum Bar {
		MID, LOW
	}
	public ClimbCommand(Climb climb, Bar e) {
		this.climb = climb;
		this.bar = e;
		addRequirements(climb);
	}

	// Called when the command is first scheduled.
	@Override
	public void initialize() {}

	// Called at 50hz while the command is scheduled.
	@Override
	public void execute() {
		if (bar == MID){
			if (climb.getEncoderValue() != Constants.CLIMBER_ENCODER_DISTANCE_MID) {
				climb.setSetpoint(Constants.CLIMBER_ENCODER_DISTANCE_MID);
			} else {
				climb.stop();
			}
		}else if (bar == LOW){
			if (climb.getEncoderValue() != Constants.CLIMBER_ENCODER_DISTANCE_LOW) {
				climb.setSetpoint(Constants.CLIMBER_ENCODER_DISTANCE_LOW);
			} else {
				climb.stop();
			}
		}
	}

	// Called once when the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		climb.stop();
	}

	@Override
	public boolean isFinished() {
		return false;
	}

}
