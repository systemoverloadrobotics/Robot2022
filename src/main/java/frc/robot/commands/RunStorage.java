package frc.robot.commands;

import javax.swing.SortOrder;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Storage;
import frc.robot.subsystems.Tank;
import frc.robot.subsystems.Storage.ToggleState;

public class RunStorage extends CommandBase {
  private Storage storage;

  public RunStorage(Storage storage) {
    this.storage = storage;
    addRequirements(storage);
  }

  // Called when the command is first scheduled.
  @Override
  public void initialize() {}

  // Called at 50hz while the command is scheduled.
  @Override
  public void execute() {
    storage.toggleBelt(ToggleState.ON);
  }

  // Called once when the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    storage.testBelt(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
