package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.constraint.SwerveDriveKinematicsConstraint;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.modules.SwerveModule;

public class Swerve extends SubsystemBase {

  //Motor Initialization
  //Front Left
  private TalonFX frontLeftPower = new TalonFX(Constants.Motor.SWERVE_FRONT_LEFT_POWER);
  private TalonSRX frontLeftSteer = new TalonSRX(Constants.Motor.SWERVE_FRONT_LEFT_STEER);
  //Front Right
  private TalonFX frontRightPower = new TalonFX(Constants.Motor.SWERVE_FRONT_RIGHT_POWER);
  private TalonSRX frontRightSteer = new TalonSRX(Constants.Motor.SWERVE_FRONT_RIGHT_STEER);
  //Back Left
  private TalonFX backLeftPower = new TalonFX(Constants.Motor.SWERVE_BACK_LEFT_POWER);
  private TalonSRX backLeftSteer = new TalonSRX(Constants.Motor.SWERVE_BACK_LEFT_STEER);
  //Back Right
  private TalonFX backRightPower = new TalonFX(Constants.Motor.SWERVE_BACK_RIGHT_POWER);
  private TalonSRX backRightSteer = new TalonSRX(Constants.Motor.SWERVE_BACK_RIGHT_POWER);

  //Swerve Modules
  private SwerveModule frontLeft;
  private SwerveModule frontRight;
  private SwerveModule backLeft;
  private SwerveModule backRight;

  //Gyro
  private PigeonIMU gyro = new PigeonIMU(Constants.Sensor.SWERVE_GYRO);

  public Swerve() {

    new Thread(() -> {
      try{
        Thread.sleep(1000);
        resetHeading();
      } catch (Exception e){}
    }).start();

    // Create four modules with correct controllers, add to modules
    frontLeft = new SwerveModule(frontLeftPower, frontLeftSteer);
    frontRight = new SwerveModule(frontRightPower, frontRightSteer);
    backLeft = new SwerveModule(backLeftPower, backLeftSteer);
    backRight = new SwerveModule(backRightPower, backRightSteer);
    
  }

  public void drive(double x1, double y1, double x2) {
    final double newY1 = (-y1 * Math.cos(gyro.getYaw())) + (x1 * Math.sin(gyro.getYaw()));
    x1 = (-y1 * Math.sin(gyro.getYaw())) + (x1 * Math.cos(gyro.getYaw()));
    y1 = -newY1;

    double r = Math
            .sqrt(Math.pow(Constants.RobotDimensions.LENGTH, 2) + Math.pow(Constants.RobotDimensions.WIDTH, 2)) / 2;

    // STR = x1, FWD = -y1
    double a = x1 - (x2 * (Constants.RobotDimensions.LENGTH / r));
    double b = x1 + (x2 * (Constants.RobotDimensions.LENGTH / r)); 
    double c = -y1 - (x2 * (Constants.RobotDimensions.WIDTH / r));
    double d = -y1 + (x2 * (Constants.RobotDimensions.WIDTH / r));

    //speed
    double ws1 = Math.sqrt(Math.pow(b, 2) + Math.pow(c, 2));
    double ws2 = Math.sqrt(Math.pow(b, 2) + Math.pow(d, 2));
    double ws3 = Math.sqrt(Math.pow(a, 2) + Math.pow(d, 2));
    double ws4 = Math.sqrt(Math.pow(a, 2) + Math.pow(c, 2));

    //check each speed


    double max = Math.max(ws1, Math.max(ws2, Math.max(ws3, ws4)));
    if (max > 1){
        ws1 /= max;
        ws2 /= max;
        ws3 /= max;
        ws4 /= max;
    }

    //Set each wheel to an angle and speed
    backLeft.setSteerRotation((Math.atan2(b, c) * (180 / Math.PI)));
    backLeft.setVelocity(ws3);
    backRight.setSteerRotation((Math.atan2(b, d) * (180 / Math.PI)));
    backRight.setVelocity(ws4);
    frontLeft.setSteerRotation((Math.atan2(a, d) * (180 / Math.PI)));
    frontLeft.setVelocity(ws1);
    frontRight.setSteerRotation((Math.atan2(a, c) * (180 / Math.PI)));
    frontRight.setVelocity(ws2);
  }

  public void stopModules(){
    frontLeft.stop();
    frontRight.stop();
    backLeft.stop();
    backRight.stop();
  }

  public void resetHeading(){
    gyro.setYaw(0, 50);
  }

  public double getHeading(){
    return Math.IEEEremainder(gyro.getYaw(), 360);
  }

  public Rotation2d getRotation2d(){
    return Rotation2d.fromDegrees(getHeading());
  }

  public void setModuleStates(SwerveModuleState[] desiredStates){
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Motor.SWERVE_MAX_SPEED);
    frontLeft.setState(desiredStates[0]);
    frontRight.setState(desiredStates[1]);
    backLeft.setState(desiredStates[2]);
    backRight.setState(desiredStates[3]);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    frontLeft.periodic();
    frontRight.periodic();
    backLeft.periodic();
    backRight.periodic();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

}
