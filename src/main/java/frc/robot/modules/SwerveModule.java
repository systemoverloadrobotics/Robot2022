package frc.robot.modules;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.fasterxml.jackson.databind.jsontype.impl.StdTypeResolverBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.util.Utils;

// SwerveModule manages an individual swerve drive module on the robot.
public class SwerveModule {
    // Should contain the two motor controllers, tracking, methods for setting,
    // and encoder output.

    private TalonFX powerController;
    private TalonSRX steerController;

    private PIDController steeringPIDController;

    public SwerveModule(TalonFX powerController, TalonSRX steerController) {
        this.powerController = powerController;
        this.steerController = steerController;

        steeringPIDController = new PIDController(Constants.PID.P_SWERVE, 0, 0);
        steeringPIDController.enableContinuousInput(-Math.PI, Math.PI);

        powerController.configFactoryDefault(); 
        steerController.configFactoryDefault();

        powerController.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        steerController.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);

    }

    public void setSteerRotation(double angle){
        steerController.set(ControlMode.Position, Utils.degreesToTicks(angle, 4096));
    }

    public double getSteerPosition(){
        return Utils.ticksToDegrees(steerController.getSelectedSensorPosition(), 4096);
    }

    public void setVelocity(double percent){
        powerController.set(ControlMode.PercentOutput, percent);
    }

    public double getVelocity(){
        return powerController.getSelectedSensorVelocity();
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState( getVelocity(), new Rotation2d(getSteerPosition()));
    }

    public void setState(SwerveModuleState state){
        if(Math.abs(state.speedMetersPerSecond) < 0.001){
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);
        powerController.set(ControlMode.PercentOutput, state.speedMetersPerSecond/ Constants.Motor.SWERVE_MAX_SPEED);
        steerController.set(ControlMode.Position, Utils.degreesToTicks(steeringPIDController.calculate(getSteerPosition(), state.angle.getDegrees()), 4096));
    }

    public void stop(){
        powerController.set(ControlMode.PercentOutput, 0);
        steerController.set(ControlMode.PercentOutput, 0);
    }
  
    public void periodic() {
        // Called at 50hz.
    }
}
