package frc.robot.modules;

import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class ClimbModule {

	private Spark spark;

	public ClimbModule(int channel, boolean isInverted) {
		spark = new Spark(channel);
		spark.setInverted(isInverted);
	}

	public void climb() {
		// set the motor expiration in seconds
		spark.setExpiration(4);
		// set motor speed
		spark.set(0.5);
		spark.stopMotor();
	}

	public void recall() {
		spark.setExpiration(4);
		spark.set(-0.5);
		spark.stopMotor();
	}
	
}
