package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.Utils;
import static java.lang.Math.*;
import static frc.robot.util.Utils.*;

public class Shooter extends SubsystemBase {
	// private WPI_TalonFX shooterMotor;
	private CANSparkMax followShooter;
	private CANSparkMax masterShooter;
	private Encoder encoder;

	public Shooter() {
		followShooter = new CANSparkMax(Constants.Motor.SHOOTER_PORT, MotorType.kBrushless);
		masterShooter = new CANSparkMax(Constants.Motor.SHOOTER_PORT, MotorType.kBrushless);
		encoder = new Encoder(0, 1);
		followShooter.follow(masterShooter);
		masterShooter.getPIDController().setP(Constants.PID.SHOOTER_MOTOR_P); 
		masterShooter.getPIDController().setI(Constants.PID.SHOOTER_MOTOR_I); 
		masterShooter.getPIDController().setD(Constants.PID.SHOOTER_MOTOR_D); 
	}

	public int getMasterMotorRPM() {
		// Math#round is used to account for floating point errors
		return (int) Math.round(masterShooter.get() * Constants.FALCON_MAX_RPM);
	}

	public void spool(int rpm) {
		masterShooter.set(rpm / 6380D);
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

	public void stopMotor() {
		masterShooter.set(0);
	}
}
