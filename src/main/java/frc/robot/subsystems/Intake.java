// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import java.sql.Time;
import java.util.concurrent.ScheduledFuture;
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

    solenoid.set(false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void intakeBall(double speed) {
    intake.set(ControlMode.PercentOutput, speed); 
  }

  public void toggleSolenoid(){
    solenoid.toggle();
  }
}