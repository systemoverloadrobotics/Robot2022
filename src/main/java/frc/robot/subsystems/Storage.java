package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Storage extends SubsystemBase {

  private TalonFX movementBelt;
  private Spark feeder;
  private Encoder feederEncoder;
  private PIDController feederController;
  private ColorSensorV3 colorSensor;
  private DigitalInput intakeSensor, storageSensor, shooterSensor;

  public Storage() {
    // Motor
    this.movementBelt = new TalonFX(Constants.Motor.STORAGE_MOVEMENT_BELT);
    this.feeder = new Spark(Constants.Motor.STORAGE_FEEDER);
    this.feederEncoder = new Encoder(Constants.Sensor.FEEDER_ENCODER_CHANNEL_A,
        Constants.Sensor.FEEDER_ENCODER_CHANNEL_B);
    this.feederController = new PIDController(Constants.PID.P_FEEDER, 0, 0);
    feederEncoder.setDistancePerPulse(Constants.RobotDimensions.FEEDER_ENCODER_DISTANCE_PER_PULSE);
    // Color Sensor
    this.colorSensor = new ColorSensorV3(I2C.Port.kOnboard);
    // Proximity Sensor
    this.intakeSensor = new DigitalInput(Constants.Sensor.PROXIMITY_INTAKE_SENSOR);
    this.storageSensor = new DigitalInput(Constants.Sensor.PROXIMITY_STORAGE_SENSOR);
    this.shooterSensor = new DigitalInput(Constants.Sensor.PROXIMITY_SHOOTER_SENSOR);
  }

  public void toggleBelt(ToggleState state) {
    switch (state) {
      case ON:
        movementBelt.set(ControlMode.PercentOutput, Constants.Motor.STORAGE_ON);
        break;
      case OFF:
        movementBelt.set(ControlMode.PercentOutput, 0);
      case REVERSE:
        movementBelt.set(ControlMode.PercentOutput, Constants.Motor.STORAGE_REVERSE);
    }
  }

  public void setFeederPos(double pos) {
    resetFeederEncoder();
    if (feederEncoder.getDistance() < pos) {
      feeder.set(feederController.calculate(feederEncoder.getDistance(), pos));
    } else {
      feeder.set(0);
    }
  }

  public BallColor getColor() {
    if (colorSensor.getRed() - 192 > 192 && colorSensor.getRed() < 256)
      return BallColor.RED;
    if (colorSensor.getBlue() > 192 && colorSensor.getBlue() < 256)
      return BallColor.BLUE;
    return BallColor.NONE;
  }

  public boolean detectBall(ProximitySensors e) {
    switch (e) {
      case INTAKE:
        return intakeSensor.get();
      case STORAGE:
        return storageSensor.get();
      case SHOOTER:
      default:
        return shooterSensor.get();
    }
  }

  public void spinFeeder() {
    feeder.set(Constants.Motor.STORAGE_FEEDER_ON);
  }

  public void reverseFeeder() {
    feeder.set(Constants.Motor.STORAGE_FEEDER_REVERSE);
  }

  public void stopFeeder() {
    feeder.stopMotor();
  }

  public void resetFeederEncoder() {
    feederEncoder.reset();
  }

  @Override
  public void periodic() {

  }

  @Override
  public void simulationPeriodic() {

  }

  public static enum ToggleState {
    ON, OFF, REVERSE;
  }

  public static enum BallColor {
    RED, BLUE, NONE;
  }

  public static enum ProximitySensors {
    INTAKE, STORAGE, SHOOTER;
  }

}
