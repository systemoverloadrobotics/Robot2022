package frc.robot.subsystems;

import java.lang.reflect.Field;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.BasePigeonSimCollection;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.modules.SwerveModule;
import frc.robot.util.ConstantButton;

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
  private TalonSRX backRightSteer = new TalonSRX(Constants.Motor.SWERVE_BACK_RIGHT_STEER);

  //Swerve Modules
  private SwerveModule frontLeft;
  private SwerveModule frontRight;
  private SwerveModule backLeft;
  private SwerveModule backRight;

  //Gyro
  private WPI_PigeonIMU gyro = new WPI_PigeonIMU(Constants.Sensor.SWERVE_GYRO);
  private BasePigeonSimCollection gyroSim;
  //Odometer
  private SwerveDriveOdometry odometry = new SwerveDriveOdometry(Constants.Motor.SWERVE_DRIVE_KINEMATICS, new Rotation2d(0));

  private Field2d field2d = new Field2d();

  public Swerve() {
    // Create four modules with correct controllers, add to modules
    frontLeft = new SwerveModule(frontLeftPower, frontLeftSteer, 2397);
    frontRight = new SwerveModule(frontRightPower, frontRightSteer, 782);
    backLeft = new SwerveModule(backLeftPower, backLeftSteer, 473);
    backRight = new SwerveModule(backRightPower, backRightSteer, 1485);

    resetHeading();
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

  public Rotation2d getRotation2d(){
    return Rotation2d.fromDegrees(Math.IEEEremainder(-gyro.getYaw(), 360));
  }

  public Pose2d getPose(){
    return odometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose){
    odometry.resetPosition(pose, getRotation2d());
  }

  public SwerveModuleState[] getModuleStates(){
    return new SwerveModuleState[] {
      frontLeft.getState(),
      frontRight.getState(),
      backLeft.getState(),
      backRight.getState()
    };
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

    SmartDashboard.putData("Gyro", gyro);
    
    odometry.update(getRotation2d(), getModuleStates());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

}
