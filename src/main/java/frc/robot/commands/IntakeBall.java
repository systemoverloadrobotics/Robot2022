// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Storage;
import frc.robot.subsystems.Storage.ProximitySensors;
import frc.robot.subsystems.Storage.ToggleState;

public class IntakeBall extends CommandBase {
  Intake intake;
  Storage storage;
  double startingFeederPos;

  long startTime;
  boolean isIdled;

  /** Creates a new IntakeBall. */
  public IntakeBall(Intake i, Storage storage) {
    this.intake = i;
    this.storage = storage;
    startingFeederPos = storage.getFeederPos();
    addRequirements(intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = System.currentTimeMillis();
    intake.actuate();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!isIdled && System.currentTimeMillis() - startTime > 400) {
      isIdled = true;
      intake.idle();
    }

    intake.intakeBall(Constants.Motor.INTAKE_SPEED);
    if (storage.detectBall(ProximitySensors.SHOOTER)) {
      double neededAmount = startingFeederPos + 2;
      if (storage.getFeederPos() >= neededAmount * 0.98 && storage.getFeederPos() <= neededAmount * 1.02) {
        storage.setFeederPos(startingFeederPos + 2);
      }
      else if (storage.detectBall(ProximitySensors.STORAGE)) {
        storage.toggleBelt(ToggleState.OFF);
      }
    }
    storage.toggleBelt(ToggleState.ON);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.retract();
    isIdled = false;
    intake.intakeBall(0);
    storage.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
