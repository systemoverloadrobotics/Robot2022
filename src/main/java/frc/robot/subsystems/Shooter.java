package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
	private WPI_TalonFX shooterMotor;

	public Shooter() {
		shooterMotor = new WPI_TalonFX(Constants.Motor.SHOOTER_PORT);
		shooterMotor.config_kP(Constants.PID.SHOOTER_P_SLOT_INDEX, Constants.PID.SHOOTER_P);
		shooterMotor.config_kI(Constants.PID.SHOOTER_I_SLOT_INDEX, Constants.PID.SHOOTER_I);
		shooterMotor.config_kD(Constants.PID.SHOOTER_D_SLOT_INDEX, Constants.PID.SHOOTER_D);
	}

	public int shooterMotorRPM() {
		// Math#round is used to account for floating point errors
		return (int) Math.round(shooterMotor.get() * Constants.FALCON_MAX_RPM);
	}

	public void spool(int rpm) {
		shooterMotor.set(ControlMode.PercentOutput, rpm / 6380D);
	}

	public void stopMotor() {
		shooterMotor.set(ControlMode.PercentOutput, 0);
	}
}