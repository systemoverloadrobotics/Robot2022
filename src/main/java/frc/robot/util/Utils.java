package frc.robot.util;

import java.util.function.Consumer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class Utils {
	public static double ticksToDegrees(double ticks, double ticksPerRotation) {
		return (ticks / ticksPerRotation ) * 360;
	}

	public static double degreesToTicks(double angle, double ticksPerRotation) {
		return (angle / 360d) * ticksPerRotation;
	}


	public static double sec(double angle) {
		return 1D / Math.cos(angle);
	}

	public static double csc(double angle) {
		return 1D / Math.sin(angle);
	}
}