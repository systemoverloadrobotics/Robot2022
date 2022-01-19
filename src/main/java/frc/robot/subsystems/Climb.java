package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climb extends SubsystemBase {
	// test motor inverts later
	private CANSparkMax followerMotor;
	private CANSparkMax mainMotor;

	private static RelativeEncoder encoder;

	public Climb() {
		mainMotor = new CANSparkMax(Constants.Motor.RIGHT_CLIMB_MOTOR, MotorType.kBrushless);
		followerMotor = new CANSparkMax(Constants.Motor.LEFT_CLIMB_MOTOR, MotorType.kBrushless);
		encoder = mainMotor.getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature, 4096);
		followerMotor.follow(mainMotor);
	}

	public static double getEncoderValue() {
		return encoder.getPosition();
	}

	public void move() {
		if(getEncoderValue() < Constants.CLIMBER_ENCODER_DISTANCE) {
			mainMotor.set(Constants.MotorSettings.MOTOR_VELOCITY);
		} else { 
			mainMotor.set(-(Constants.MotorSettings.MOTOR_VELOCITY));
		}
	}
}
