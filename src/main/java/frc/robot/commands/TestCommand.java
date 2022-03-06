package frc.robot.commands;

import javax.swing.SortOrder;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Storage;
import frc.robot.subsystems.Tank;
import frc.robot.subsystems.Storage.ToggleState;

public class TestCommand extends CommandBase {
  private  Intake intake;
  private  Storage storage;
  private  Shooter shooter;

  public TestCommand(Intake intake, Storage storage, Shooter shooter) {
    this.intake = intake;
    this.storage = storage;
    this.shooter = shooter;
    addRequirements(intake, storage, shooter);
  }

  // Called when the command is first scheduled.
  @Override
  public void initialize() {}

  // Called at 50hz while the command is scheduled.
  @Override
  public void execute() {
    intake.intakeBall(-0.75);
    storage.toggleBelt(ToggleState.ON);
    storage.testBelt(-0.8);
    shooter.spool(0.5);

  }

  // Called once when the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.intakeBall(0);
    storage.testBelt(0);;
    storage.stopFeeder();
    shooter.spool(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
