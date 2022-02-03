// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Storage;
import frc.robot.subsystems.Storage.ProximitySensors;
import frc.robot.subsystems.Storage.ToggleState;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class StorageCommand extends CommandBase {
  private final Storage storage;

  public StorageCommand(Storage storage) {
    this.storage = storage;
    addRequirements(storage);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    while(!storage.detectBall(ProximitySensors.STORAGE)){
      if(storage.detectBall(ProximitySensors.INTAKE)){
        storage.toggleBelt(ToggleState.ON);
      }
    }
    if(IndexBall.isBallInFeeder){
      storage.toggleBelt(ToggleState.OFF);
    }
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
