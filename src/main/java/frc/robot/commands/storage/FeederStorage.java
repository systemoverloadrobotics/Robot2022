package frc.robot.commands.storage;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Storage;
import frc.robot.subsystems.Storage.ProximitySensors;
import frc.robot.subsystems.Storage.ToggleState;

public class FeederStorage extends CommandBase{
  private final Storage storage;

  public FeederStorage(Storage storage) {
    this.storage = storage;
    addRequirements(storage);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    storage.toggleBelt(ToggleState.ON);
    while(!storage.detectBall(ProximitySensors.SHOOTER)){
      if(storage.detectBall(ProximitySensors.STORAGE) && !IndexBall.isBallInFeeder){
        storage.spinFeeder();
        IndexBall.isBallInFeeder = true;
      }else{
        storage.toggleBelt(ToggleState.OFF);
      }
    }
    storage.setFeederPos(Constants.RobotDimensions.FEEDER_OFFSET_DISTANCE);
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
