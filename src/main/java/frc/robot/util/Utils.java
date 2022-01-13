package frc.robot.core;

public class Utils {
	public static double degreesToTicks(double degrees) {
		return (degrees / 360) * 2048;
	}
}