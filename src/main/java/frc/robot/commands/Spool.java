package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Storage;
import frc.robot.subsystems.Storage.ToggleState;

public class Spool extends CommandBase {
  private final Shooter shooter;
  private final Storage storage;
  public Spool(Shooter shooter, Storage storage) {
    this.shooter = shooter;
    this.storage = storage;
    addRequirements(shooter);
  }

  // Called when the command is first scheduled.
  @Override
  public void initialize() {}

  // Called at 50hz while the command is scheduled.
  @Override
  public void execute() {
    shooter.spool(true);
    storage.toggleBelt(ToggleState.ON);
  }

  // Called once when the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.spool(false);
    storage.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}

