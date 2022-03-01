package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Storage;
import frc.robot.subsystems.Storage.ToggleState;

public class IndexBall extends SequentialCommandGroup {
  private Storage storage;

  public static boolean isBallInFeeder;

  public IndexBall(Storage storage) {
    this.storage = storage;
    addRequirements(storage);
  }

  public void indexBall() {
    // no ball inside the robot or one ball is stored in the shooter
    addCommands(
      parallel(
        new StorageCommand(storage),
        new FeederStorage(storage)
      )
    );
  }

  // Called when the command is first scheduled.
  @Override
  public void initialize() {
    isBallInFeeder = false;
  }

  // Called at 50hz while the command is scheduled.
  @Override
  public void execute() {
    indexBall();
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
