package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.Intake;

public class ActuateIntake extends CommandBase {
	private Intake intake;
	private Joystick joystick;

	public ActuateIntake(Intake intake) {
		this.intake = intake;
		addRequirements(intake);
	}

	@Override
	public void execute() {
		intake.toggleSolenoid();
	}

	@Override
	public void initialize() {}

	@Override
	public void end(boolean interrupted) {}

	@Override
	public boolean isFinished() {
		return false;
	}
}
