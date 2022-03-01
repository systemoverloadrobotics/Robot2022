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
  private Solenoid solenoid;

  /** Creates a new Intake. */
  public Intake() {
    intake = new TalonFX(Constants.Motor.INTAKE);
    solenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 1);

    disableSolenoid();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void intakeBall(double speed) {
    intake.set(ControlMode.PercentOutput, speed);
  }

  public void reverseIntake() {
    intake.set(ControlMode.PercentOutput, Constants.Motor.INTAKE_REVERSE);
  }

  public void enableSolenoid() {
    solenoid.set(true);
  }

  public void disableSolenoid() {
    solenoid.set(false);
  }
}
