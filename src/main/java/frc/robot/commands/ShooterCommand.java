package frc.robot.commands;


import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Storage;


public class ShooterCommand extends CommandBase {
	private Storage storage;
	private Shooter shooter;

	public ShooterCommand(Storage storage, Shooter shooter) {
		this.storage = storage;
		this.shooter = shooter;
		addRequirements(storage, shooter);
	}

	// Called when the command is first scheduled.
	@Override
	public void initialize() {

	}

	// Called at 50hz while the command is scheduled.
	@Override
	public void execute() {
		shooter.getRPM(); 
		shooter.spool(true);
		if (shooter.getRPM() >= (-3370 * 0.95)){
			storage.spinFeeder();
			storage.testBelt(-0.8);
		}
		if (DriverStation.isAutonomous() && DriverStation.getMatchTime() == 7.5) {
			end(true);
		}
	}

	// Called once when the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		shooter.spool(false);
		storage.stopFeeder();
		storage.testBelt(0);
		//storage.toggleBelt(ToggleState.OFF);
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
