package frc.robot.commands;


import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Storage;
import frc.robot.subsystems.Storage.ToggleState;


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
		shooter.spool(true);
		if (shooter.getRPM() >= (Constants.SHOOTER_DESIRED_RPM * 0.95) && shooter.getRPM() <= (Constants.SHOOTER_DESIRED_RPM * 1.05)){
			storage.spinFeeder(false);
			storage.toggleBelt(ToggleState.ON);
		}
		if (DriverStation.isAutonomous() && DriverStation.getMatchTime() >= 7.5) {
			end(true);
		}
	}

	// Called once when the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		shooter.spool(false);
		storage.stop();
	}

	@Override
	public boolean isFinished(){
		return false;
	}
}
