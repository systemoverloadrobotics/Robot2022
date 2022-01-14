package frc.robot.util;

public class Utils {
	public static double ticksToDegrees(double ticks, double ticksPerRotation) {
		return (ticks / ticksPerRotation ) * 360;
	}

	public static double degreesToTicks(double angle, double ticksPerRotation) {
		return (angle / 360d) * ticksPerRotation;
	}
}