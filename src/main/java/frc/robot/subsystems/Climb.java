package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climb extends SubsystemBase {
	// test motor inverts later
	private CANSparkMax followerMotor;
	private CANSparkMax mainMotor;

	private RelativeEncoder encoder;
	private SparkMaxPIDController pidController; 


	public Climb() {
		mainMotor = new CANSparkMax(Constants.Motor.RIGHT_CLIMB_MOTOR, MotorType.kBrushless);
		followerMotor = new CANSparkMax(Constants.Motor.LEFT_CLIMB_MOTOR, MotorType.kBrushless);
		encoder = mainMotor.getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature, 4096);
		pidController = mainMotor.getPIDController();
		pidController.setP(Constants.PID.P_CLIMB); 
		pidController.setI(Constants.PID.I_CLIMB); 
		pidController.setD(Constants.PID.D_CLIMB); 
		followerMotor.follow(mainMotor);
	}
	//manually moves the climb bar up
	public void overrideUp(){
		mainMotor.set(ControlMode.Velocity, 100);
	}

	//Manually moves the climb bar down
	public void overrideDown(){
		mainMotor.set(ControlMode.PercentOutput, -100);
	}

	public double getEncoderValue() {
		return encoder.getPosition();
	}

	public void setSetpoint(double setpoint) {
		pidController.setReference(setpoint, CANSparkMax.ControlType.kPosition);
	}

	public void stop() {
		mainMotor.stopMotor();
	}
}
