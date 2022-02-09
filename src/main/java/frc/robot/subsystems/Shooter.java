package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.fasterxml.jackson.annotation.JsonFormat.Shape;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
	private CANSparkMax shooterMotor;

	public Shooter() {
		shooterMotor = new CANSparkMax(Constants.Motor.SHOOTER_PORT, MotorType.kBrushless);
		shooterMotor.getPIDController().setP(Constants.PID.SHOOTER_P);
		shooterMotor.getPIDController().setI(Constants.PID.SHOOTER_I);
		shooterMotor.getPIDController().setD(Constants.PID.SHOOTER_D);
	}

	public int shooterMotorRPM() {
		// Math#round is used to account for floating point errors
		return (int) Math.round(shooterMotor.get() * Constants.FALCON_MAX_RPM);
	}

	public void spool(double rpm) {
		shooterMotor.set(rpm / 5676D);
	}

	public void stopMotor() {
		shooterMotor.set(0);
	}
}
