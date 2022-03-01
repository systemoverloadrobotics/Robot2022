package frc.robot.commands.storage;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Storage;
import frc.robot.subsystems.Storage.ToggleState;

public class IndexBall extends CommandBase {
  private Storage storage;

  public static boolean isBallInFeeder;

  public IndexBall(Storage storage) {
    this.storage = storage;
    addRequirements(storage);
  }

  // Called when the command is first scheduled.
  @Override
  public void initialize() {
    isBallInFeeder = false;
  }

  // Called at 50hz while the command is scheduled.
  @Override
  public void execute() {
    new ParallelCommandGroup(
      new StorageCommand(storage),
      new FeederStorage(storage)
    );
  }

  // Called once when the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // making sure that the motors have stopped
    storage.toggleBelt(ToggleState.OFF);
    storage.stopFeeder();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
