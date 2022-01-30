package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Storage;
import frc.robot.subsystems.Storage.ToggleState;
import frc.robot.util.ConstantButton;

public class IndexBall extends CommandBase{
  
  private Intake intake;
  private Storage storage;
  
  public IndexBall(Intake intake, Storage storage) {
    this.intake = intake;
    this.storage = storage;
    addRequirements(intake, storage);
  }

  // Called when the command is first scheduled.
  @Override
  public void initialize() {
    intake.toggleSolenoid();
  }

  // Called at 50hz while the command is scheduled.
  @Override
  public void execute() {
    intake.intakeBall(Constants.Motor.INTAKE_SPEED);
    if(storage.detectBall(0)){
      storage.toggleBelt(ToggleState.ON);
      if(storage.detectBall(1) ){
        storage.spinFeeder();
        if(storage.detectBall(2) && !storage.detectBall(1)){
          storage.stopFeeder();
          storage.setFeederPos(Constants.RobotDimensions.FEEDER_OFFSET_DISTANCE);
        }
      }
    }
    
  }

  // Called once when the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.toggleSolenoid();
    intake.intakeBall(0);
    storage.toggleBelt(ToggleState.OFF);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
