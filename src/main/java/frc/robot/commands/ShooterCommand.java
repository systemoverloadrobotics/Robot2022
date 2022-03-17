package frc.robot.commands;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Storage;
import frc.robot.subsystems.Swerve;
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
		double rpm = -3770;
		shooter.spool(true);
		if (shooter.getRPM() >= rpm * 0.95){
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
