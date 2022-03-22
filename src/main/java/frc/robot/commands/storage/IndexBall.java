package frc.robot.commands.storage;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.IntakeBall;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Storage;
import frc.robot.subsystems.Storage.ToggleState;

public class IndexBall extends CommandBase {
  private Storage storage;
  private Intake intake;

  public static boolean isBallInFeeder;

  public IndexBall(Storage storage, Intake intake) {
    this.storage = storage;
    this.intake = intake;
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
      new IntakeBall(intake, storage),
      new FeederStorage(storage)
    );
  }

  // Called once when the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // making sure that the motors have stopped
    storage.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
