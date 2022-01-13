package frc.robot.util;

public class Utils {
	public static double tickToDegrees(double ticks, double ticksPerRotation) {
		return (ticks / ticksPerRotation ) * 360;
	}
}