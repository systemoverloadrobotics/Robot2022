package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
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

  //Odometer
  private SwerveDriveOdometry odometry = new SwerveDriveOdometry(Constants.Motor.SWERVE_DRIVE_KINEMATICS, new Rotation2d(0));

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

  public Pose2d getPose(){
    return odometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose){
    odometry.resetPosition(pose, getRotation2d());
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
