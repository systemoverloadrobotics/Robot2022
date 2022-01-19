package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climb extends SubsystemBase {
	// test motor inverts later
	private CANSparkMax rightClimbMotor;
	private CANSparkMax leftClimbMotor;

	private RelativeEncoder leftEncoder;
	private RelativeEncoder righEncoder;

	public Climb() {
		rightClimbMotor = new CANSparkMax(Constants.Motor.RIGHT_CLIMB_MOTOR, MotorType.kBrushless);
		leftClimbMotor = new CANSparkMax(Constants.Motor.LEFT_CLIMB_MOTOR, MotorType.kBrushless);
		leftEncoder = leftClimbMotor.getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature, 4096);
		righEncoder = rightClimbMotor.getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature, 4096);
		rightClimbMotor.follow(leftClimbMotor);
	}

	public double getEncoderValue() {
		return leftEncoder.getPosition();
	}

	public void climb() {
		// set the motor expiration in seconds

		// set motor speed
		leftClimbMotor.set(0.5);
		leftClimbMotor.stopMotor();
	}



	public void recall() {
		leftClimbMotor.set(-0.5);
		leftClimbMotor.stopMotor();

	}
	// public Climb() {
	// leftMotor = new ClimbModule(Constants.Motor.LEFT_CLIMB_MOTOR, false);
	// rightMotor = new ClimbModule(Constants.Motor.RIGHT_CLIMB_MOTOR, true);
	// }

	// public void climbUp() {
	// leftMotor.climb();
	// rightMotor.climb();
	// }

	// public void recall() {
	// leftMotor.recall();
	// rightMotor.recall();
	// }

}
