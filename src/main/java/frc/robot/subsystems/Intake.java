// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

  private TalonFX intake;
  private Solenoid leftSolenoid;
  private Solenoid rightSolenoid;

  /** Creates a new Intake. */
  public Intake() {
    intake = new TalonFX(Constants.Motor.INTAKE);
    leftSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 0);
    rightSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 1);
    disableSolenoid();
  }

  public void intakeBall(double speed) {
    intake.set(ControlMode.PercentOutput, speed);
  }

  public void reverseIntake() {
    intake.set(ControlMode.PercentOutput, Constants.Motor.INTAKE_REVERSE);
  }

  public void enableSolenoid() {
    leftSolenoid.set(true);
    rightSolenoid.set(true);
  }

  public void disableSolenoid() {
    leftSolenoid.set(false);
    rightSolenoid.set(false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
}
