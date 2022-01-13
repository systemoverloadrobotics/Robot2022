package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;
import java.util.function.DoubleSupplier;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
import org.w3c.dom.views.DocumentView;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.modules.SwerveModule;

public class Swerve extends SubsystemBase {

  //Motor Initialization

  //Front Left
  private TalonFX frontLeftPower = new TalonFX(0);
  private TalonSRX frontLeftSteer = new TalonSRX(1);
  //Front Right
  private TalonFX frontRightPower = new TalonFX(2);
  private TalonSRX frontRightSteer = new TalonSRX(3);
  //Back Left
  private TalonFX backLeftPower = new TalonFX(4);
  private TalonSRX backLeftSteer = new TalonSRX(5);
  //Back Right
  private TalonFX backRightPower = new TalonFX(6);
  private TalonSRX backRightSteer = new TalonSRX(7);

  //Swerve Modules
  private SwerveModule frontLeft;
  private SwerveModule frontRight;
  private SwerveModule backLeft;
  private SwerveModule backRight;

  //Gyro
  private PigeonIMU gyro = new PigeonIMU(8);

  public static enum Corner {
    FRONT_LEFT,
    FRONT_RIGHT,
    BACK_LEFT,
    BACK_RIGHT,
  }

  private Map<Corner, SwerveModule> modules = new HashMap<>();

  public Swerve() {
    // Create four modules with correct controllers, add to modules
    frontLeft = new SwerveModule(frontLeftPower, frontLeftSteer);
    frontRight = new SwerveModule(frontRightPower, frontRightSteer);
    backLeft = new SwerveModule(backLeftPower, backLeftSteer);
    backRight = new SwerveModule(backRightPower, backRightSteer);

    modules.put(Corner.FRONT_LEFT, frontLeft);
    modules.put(Corner.FRONT_RIGHT, frontRight);
    modules.put(Corner.BACK_LEFT, backLeft);
    modules.put(Corner.BACK_RIGHT, backRight);
  }

  public void drive(double x1, double y1, double x2){

    double temp = (-y1 * Math.cos(gyro.getYaw())) + (x1 * Math.sin(gyro.getYaw()));
    x1 = (-y1 * Math.sin(gyro.getYaw())) + (x1 * Math.cos(gyro.getYaw()));
    y1 = -temp;

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

    double max = ws1;
    if(ws2 > max) max = ws2;
    if(ws3 > max) max = ws3;
    if(ws4 > max) max = ws4;
    if(max > 1){
        ws1 /= max;
        ws2 /= max;
        ws3 /= max;
        ws4 /= max;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    for (var module : modules.values()) {
      module.periodic();
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

}
