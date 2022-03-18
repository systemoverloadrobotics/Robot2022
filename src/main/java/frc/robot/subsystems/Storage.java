package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Storage extends SubsystemBase {

  private VictorSPX movementBelt;
  private WPI_VictorSPX feeder;
  private DutyCycleEncoder feederEncoder;
  private PIDController feederController;
  //private ColorSensorV3 colorSensor;
  private DigitalInput intakeSensor, storageSensor, shooterSensor;

  public Storage() {
    // Motor
    this.movementBelt = new VictorSPX(Constants.Motor.STORAGE_MOVEMENT_BELT);
    this.feeder = new WPI_VictorSPX(Constants.Motor.STORAGE_FEEDER);
    this.feederEncoder = new DutyCycleEncoder(2);
    this.feederController = new PIDController(Constants.PID.P_FEEDER, 0, 0);
    feederEncoder.setDistancePerRotation(Constants.RobotDimensions.FEEDER_ENCODER_DISTANCE_PER_PULSE);
    // Color Sensor
    //this.colorSensor = new ColorSensorV3(I2C.Port.kOnboard);
    // Proximity Sensor
    //this.intakeSensor = new DigitalInput(Constants.Sensor.PROXIMITY_INTAKE_SENSOR);
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

  public void testBelt(double rpm){
    movementBelt.set(ControlMode.PercentOutput, rpm);
  }

  public void setFeederPos(double pos) {
    resetFeederEncoder();
    if (feederEncoder.getDistance() < pos) {
      feeder.set(ControlMode.Position, feederController.calculate(feederEncoder.getDistance(), pos));
    }
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

  public static enum ToggleState {
    ON, OFF, REVERSE;
  }

  public static enum BallColor {
    RED, BLUE, NONE;
  }

  public static enum ProximitySensors {
    INTAKE, STORAGE, SHOOTER;
  }

  @Override
  public void periodic() {

  }

  @Override
  public void simulationPeriodic() {

  }
}
