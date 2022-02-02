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
  private DigitalInput[] proximitySensor = new DigitalInput[3];

  public Storage() {
    //Motor
    this.movementBelt = new TalonFX(Constants.Motor.STORAGE_MOVEMENT_BELT);
    this.feeder = new Spark(Constants.Motor.STORAGE_FEEDER);
    this.feederEncoder = new Encoder(Constants.Sensor.FEEDER_ENCODER_CHANNEL_A, Constants.Sensor.FEEDER_ENCODER_CHANNEL_B);
    this.feederController = new PIDController(Constants.PID.P_FEEDER, 0, 0);
    feederEncoder.setDistancePerPulse(Constants.RobotDimensions.FEEDER_ENCODER_DISTANCE_PER_PULSE);
    //Color Sensor
    this.colorSensor = new ColorSensorV3(I2C.Port.kOnboard);
    //Proximity Sensor
    this.intakeSensor = new DigitalInput(Constants.Sensor.CLIMB_INTAKE_SENSOR);
    this.storageSensor = new DigitalInput(Constants.Sensor.CLIMB_STORAGE_SENSOR);
    this.shooterSensor = new DigitalInput(Constants.Sensor.CLIMB_SHOOTER_SENSOR);
    proximitySensor[0] = intakeSensor;
    proximitySensor[1] = storageSensor;
    proximitySensor[2] = shooterSensor;
  }

  public void toggleBelt(ToggleState state) {
    if (state == ToggleState.ON) {
      movementBelt.set(ControlMode.PercentOutput, 0.5);
    }
    else if (state == ToggleState.OFF) {
      movementBelt.set(ControlMode.PercentOutput, 0);
    }else if (state == ToggleState.REVERSE){
      movementBelt.set(ControlMode.Position, -0.75);
    }
  }

  public void setFeederPos(double pos){
    resetFeederEncoder();
    if(feederEncoder.getDistance() < pos){
      feeder.set(feederController.calculate(feederEncoder.getDistance(), pos));
    }else{
      feeder.set(0);
    }
  }

  public BallColor getColor() {
    if (colorSensor.getRed() > 192 && colorSensor.getRed() < 256) return BallColor.RED;
    if (colorSensor.getBlue() > 192 && colorSensor.getBlue() < 256) return BallColor.BLUE;
    return BallColor.NONE;
  }

  public boolean detectBall(int sensorID){
    return proximitySensor[sensorID].get();
  }

  public void spinFeeder(){
    feeder.set(0.5);
  }

  public void reverseFeeder(){
    feeder.set(-0.5);
  }

  public void stopFeeder(){
    feeder.stopMotor();
  }

  public void resetFeederEncoder(){
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

}
