package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.Utils;
import static java.lang.Math.*;

public class Shooter extends SubsystemBase {
	private CANSparkMax followShooter;
	private CANSparkMax masterShooter;
	private SparkMaxPIDController pidController;

	public Shooter() {
		followShooter = new CANSparkMax(Constants.Motor.SHOOTER_PORT_FOLLOWER, MotorType.kBrushless);
		masterShooter = new CANSparkMax(Constants.Motor.SHOOTER_PORT_MASTER, MotorType.kBrushless);
		pidController = masterShooter.getPIDController();
		pidController.setP(Constants.PID.SHOOTER_MOTOR_P);
		pidController.setI(Constants.PID.SHOOTER_MOTOR_I);
		pidController.setD(Constants.PID.SHOOTER_MOTOR_D);
		
		followShooter.follow(masterShooter);
	}

	public int getMasterMotorRPM() {
		// Math#round is used to account for floating point errors
		return (int) Math.round(masterShooter.get() * Constants.FALCON_MAX_RPM);
	}

	public void spool(boolean spooling) {
		if(spooling) {
			masterShooter.set(-0.67);
		} else {
			masterShooter.set(0);
		}
	}

	public double getVelocity(Limelight limelight) {
		double verticalAngleOffset = limelight.networkTable.getEntry("ty").getDouble(0);
		final double xpos = (Constants.HUB_HEIGHT - Constants.LIMELIGHT_HEIGHT)
				/ Math.tan(Constants.SHOOTER_LIMELIGHT_ANGLE + verticalAngleOffset);
		final double ypos = Constants.SHOOTER_HEIGHT;
		final double launchAngle = Math.toRadians(Constants.SHOOTER_LIMELIGHT_ANGLE);
		final double hub = Constants.HUB_HEIGHT;

		final double time = sqrt((xpos * tan(launchAngle) - 2.64 + ypos) / hub);
		final double launchSpeed = (xpos * Utils.sec(launchAngle)) / time;

		return launchSpeed;
	}

	public void setVelocity(double velocity){
		pidController.setReference(velocity, ControlType.kVelocity);
	}

	public double getRPM(){
		return masterShooter.getEncoder().getVelocity();
	}

	public void stopMotor() {
		masterShooter.set(0);
	}
}
