package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
	private WPI_TalonFX shooterMotor;

	public Shooter() {
		shooterMotor = new WPI_TalonFX(Constants.Motor.SHOOTER_PORT);
		shooterMotor.config_kP(Constants.PID.SHOOTER_P_SLOT_INDEX, Constants.PID.SHOOTER_P_VALUE);
		shooterMotor.config_kI(Constants.PID.SHOOTER_I_SLOT_INDEX, Constants.PID.SHOOTER_I_VALUE);
		shooterMotor.config_kD(Constants.PID.SHOOTER_D_SLOT_INDEX, Constants.PID.SHOOTER_D_VALUE);
	}

	public void shoot(double percentage) {
		shooterMotor.set(ControlMode.PercentOutput, percentage);
	}

	public void stopMotor() {
		shooterMotor.set(ControlMode.PercentOutput, 0);
	}
}
