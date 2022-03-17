// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import javax.print.attribute.standard.Compression;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

  private TalonFX intake;
  private DoubleSolenoid solenoid;

  /** Creates a new Intake. */
  public Intake() {
    intake = new TalonFX(Constants.Motor.INTAKE);
    solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
  }

  public void intakeBall(double speed) {
    intake.set(ControlMode.PercentOutput, speed);
  }

  public void reverseIntake() {
    intake.set(ControlMode.PercentOutput, Constants.Motor.INTAKE_REVERSE);
  }



  public void toggleSolenoid() {
    solenoid.toggle();
  }

  public void actuate(){
    solenoid.set(Value.kForward);
  }

  public void retract(){
    solenoid.set(Value.kReverse);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
}
