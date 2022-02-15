package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.drive.Vector2d;
import frc.robot.Constants;

public class Utils {
	public static double ticksToDegrees(double ticks, double ticksPerRotation) {
		return (ticks / ticksPerRotation ) * 360;
	}

	public static double degreesToTicks(double angle, double ticksPerRotation) {
		return (angle / 360d) * ticksPerRotation;
	}

	public static double sensorUnitsPer100msToMetersPerSecond(double sensorUnitsPer100ms) {
		return (sensorUnitsPer100ms * (Constants.RobotDimensions.WHEEL_CIRCUMFRENCE / 4096)) * 10;
	}

    /**
     * Gets inverse of atan2
     * @param atan2 atan2 of vector in radians
     * @param length length of vector
     * @return {@link Vector2d} class which contains x and y of vector
     */
    public static Vector2d atan2Inverse(double atan2, double length) {
        double dx = length * Math.cos(atan2);
        double dy = length * Math.sin(atan2);
		return new Vector2d(dx, dy);
	}

    public static Vector2d addVectors(Vector2d vec1, Vector2d vec2) {
		return new Vector2d(vec1.x + vec2.x, vec1.y + vec2.y);
	}

    public static Vector2d[] normalizeVectorSpeed(Vector2d[] vectors) {
        Vector2d[] out = new Vector2d[vectors.length];
        double max = Constants.Motor.SWERVE_MAX_SPEED;
        for (Vector2d vector : vectors) {
            max = Math.max(max, vector.magnitude());
        }
        
        int i = 0;
        for (Vector2d vector : vectors) {
            double scalingFactor = Math.sqrt(Constants.Motor.SWERVE_MAX_SPEED) / max;
            out[i++] = new Vector2d(vector.x * scalingFactor, vector.y * scalingFactor);
        }

		return out;
	}

    public static SwerveModuleState vectorToSwerveModuleState(Vector2d vector) {
        return new SwerveModuleState(vector.magnitude(), new Rotation2d(Math.atan2(vector.x, vector.y)));
    }

	/**
   * Minimize the change in heading the desired swerve module state would require by potentially
   * reversing the direction the wheel spins. Customized from WPILib's version to include placing
   * in appropriate scope for CTRE onboard control.
   *
   * @param desiredState The desired state.
   * @param currentAngle The current module angle.
   */
  public static SwerveModuleState optimize(SwerveModuleState desiredState, Rotation2d currentAngle) {
    double targetAngle = placeInAppropriate0To360Scope(currentAngle.getDegrees(), desiredState.angle.getDegrees());
    double targetSpeed = desiredState.speedMetersPerSecond;
    double delta = targetAngle - currentAngle.getDegrees();
    if (Math.abs(delta) > 90){
        targetSpeed = -targetSpeed;
        targetAngle = delta > 90 ? (targetAngle -= 180) : (targetAngle += 180);
    }        
    return new SwerveModuleState(targetSpeed, Rotation2d.fromDegrees(targetAngle));
  }
  
	private static double placeInAppropriate0To360Scope(double scopeReference, double newAngle) {
      double lowerBound;
      double upperBound;
      double lowerOffset = scopeReference % 360;
      if (lowerOffset >= 0) {
          lowerBound = scopeReference - lowerOffset;
          upperBound = scopeReference + (360 - lowerOffset);
      } else {
          upperBound = scopeReference - lowerOffset;
          lowerBound = scopeReference - (360 + lowerOffset);
      }
      while (newAngle < lowerBound) {
          newAngle += 360;
      }
      while (newAngle > upperBound) {
          newAngle -= 360;
      }
      if (newAngle - scopeReference > 180) {
          newAngle -= 360;
      } else if (newAngle - scopeReference < -180) {
          newAngle += 360;
      }
      return newAngle;
  }

}