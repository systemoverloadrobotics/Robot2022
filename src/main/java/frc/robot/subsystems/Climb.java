package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.modules.ClimbModule;

public class Climb extends SubsystemBase {
	// test motor inverts later
	private ClimbModule leftMotor;
	private ClimbModule rightMotor;


	public Climb() {
		leftMotor = new ClimbModule(Constants.Motor.LEFT_CLIMB_MOTOR, false);
		rightMotor = new ClimbModule(Constants.Motor.RIGHT_CLIMB_MOTOR, true);
	}

	public void climbUp() {
		leftMotor.climb();
		rightMotor.climb();
	}

	public void recall() {
		leftMotor.recall();
		rightMotor.recall();
	}

}
