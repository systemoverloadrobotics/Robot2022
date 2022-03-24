package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.storage.FeederStorage;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Storage;
import frc.robot.subsystems.Storage.ToggleState;

public class ReverseStorage extends CommandBase {
  private Storage storage;
  private Intake intake; 
  private FeederStorage feederStorage; 

  public ReverseStorage(Storage storage, Intake intake) {
    this.storage = storage;
    this.intake = intake; 
    this.feederStorage = feederStorage; 
    addRequirements(storage);
  }

  // Called when the command is first scheduled.
  @Override
  public void initialize() {}

  // Called at 50hz while the command is scheduled.
  @Override
  public void execute() {
    intake.intakeBall(1);
    storage.spinFeeder(true);
    storage.toggleBelt(ToggleState.REVERSE);
  }

  // Called once when the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    storage.toggleBelt(ToggleState.OFF);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
