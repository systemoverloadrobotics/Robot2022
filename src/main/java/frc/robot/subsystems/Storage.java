package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Storage extends SubsystemBase {
  private TalonFX movementBelt;
  private ColorSensorV3 colorSensor;

  public Storage() {
    this.movementBelt = new TalonFX(Constants.Motor.STORAGE_MOVEMENT_BELT);
    this.colorSensor = new ColorSensorV3(I2C.Port.kOnboard);
  }

  public void toggleBelt(ToggleState state) {
    if (state == ToggleState.ON) {
      movementBelt.set(ControlMode.PercentOutput, 0.5);
    }
    else if (state == ToggleState.OFF) {
      movementBelt.set(ControlMode.PercentOutput, 0);
    }
  }

  public BallColor getColor() {
    if (colorSensor.getRed() > 192 && colorSensor.getRed() < 256) return BallColor.RED;
    if (colorSensor.getBlue() > 192 && colorSensor.getBlue() < 256) return BallColor.BLUE;
    return BallColor.NONE;
  }

  @Override
  public void periodic() {
    
  }

  @Override
  public void simulationPeriodic() {

  }

  public static enum ToggleState {
    ON, OFF;
  }

  public static enum BallColor {
    RED, BLUE, NONE;
  }
}
