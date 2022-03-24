package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Storage.ToggleState;
import frc.robot.util.Utils;
import static java.lang.Math.*;

public class Shooter extends SubsystemBase {
	private CANSparkMax masterShooter;
	public SparkMaxPIDController pidController; 
	private SimpleMotorFeedforward ffController = new SimpleMotorFeedforward(0.46135, 2.4822, 0.18183);

	@Override
	public void periodic() {
		SmartDashboard.putNumber("shooter rpm", getRPM());
	}
	

	public Shooter() {
		
		masterShooter = new CANSparkMax(Constants.Motor.SHOOTER_PORT_MASTER, MotorType.kBrushless);
		masterShooter.restoreFactoryDefaults();
		pidController = masterShooter.getPIDController(); 
		masterShooter.setInverted(true);
		pidController.setP(Constants.PID.SHOOTER_MOTOR_P); 
		pidController.setI(Constants.PID.SHOOTER_MOTOR_I); 
		pidController.setD(Constants.PID.SHOOTER_MOTOR_D);
	}


public void spool(boolean spooling) {
		if(spooling) {
			pidController.setReference(ffController.calculate(Constants.SHOOTER_DESIRED_RPM), ControlType.kVelocity);
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

	public double getRPM(){
		return masterShooter.getEncoder().getVelocity();
	}

}
