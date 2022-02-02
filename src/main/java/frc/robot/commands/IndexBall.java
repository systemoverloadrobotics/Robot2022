package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Storage;
import frc.robot.subsystems.Storage.ToggleState;

public class IndexBall extends CommandBase{
  private Storage storage;

  private boolean isBallInFeeder;
  
  public IndexBall(Storage storage) {
    this.storage = storage;
    addRequirements(storage);
  }

  public void indexBall(){
    //no ball inside the robot or one ball is stored in the shooter
    if(storage.detectBall(0)){
      storage.toggleBelt(ToggleState.ON);
      if(storage.detectBall(1) && !isBallInFeeder){
        storage.spinFeeder();
        if(storage.detectBall(2)){
          storage.stopFeeder();
          storage.setFeederPos(Constants.RobotDimensions.FEEDER_OFFSET_DISTANCE);
          isBallInFeeder = true;
        }  
      }else{
        storage.toggleBelt(ToggleState.OFF);
      }
    }
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
    //making sure that the motors have stopped
    storage.toggleBelt(ToggleState.OFF);
    storage.stopFeeder();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
