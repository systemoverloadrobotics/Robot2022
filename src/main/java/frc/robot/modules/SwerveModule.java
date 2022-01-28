package frc.robot.modules;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.fasterxml.jackson.databind.jsontype.impl.StdTypeResolverBuilder;
import org.opencv.calib3d.StereoBM;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.robot.Constants;
import frc.robot.util.Utils;

// SwerveModule manages an individual swerve drive module on the robot.
public class SwerveModule {

    // Should contain the two motor controllers, tracking, methods for setting,
    // and encoder output.

    private TalonFX powerController;
    private TalonSRX steerController;

    public SwerveModule(TalonFX powerController, TalonSRX steerController) {
        this.powerController = powerController;
        this.steerController = steerController;

        powerController.configFactoryDefault(); 
        steerController.configFactoryDefault();

        powerController.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        steerController.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);

        powerController.config_kP(0, Constants.PID.P_SWERVE_POWER);
        powerController.config_kI(0, Constants.PID.I_SWERVE_POWER);
        powerController.config_kD(0, Constants.PID.D_SWERVE_POWER);

        steerController.config_kP(1, Constants.PID.P_SWERVE_STEER);
        steerController.config_kI(1, Constants.PID.D_SWERVE_STEER);
        steerController.config_kD(1, Constants.PID.I_SWERVE_STEER);


        powerController.configNominalOutputForward(Constants.Motor.SWERVE_NOMINAL_OUTPUT_PERCENT);
        powerController.configNominalOutputReverse(Constants.Motor.SWERVE_NOMINAL_OUTPUT_PERCENT);

        steerController.configNominalOutputForward(Constants.Motor.SWERVE_NOMINAL_OUTPUT_PERCENT);
        steerController.configNominalOutputReverse(Constants.Motor.SWERVE_NOMINAL_OUTPUT_PERCENT);

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
        powerController.set(ControlMode.Velocity, state.speedMetersPerSecond);
        steerController.set(ControlMode.Position, Utils.degreesToTicks(state.angle.getDegrees(), 4096));
    }

    public void stop(){
        powerController.set(ControlMode.PercentOutput, 0);
        steerController.set(ControlMode.PercentOutput, 0);
    }
  
    public void periodic() {
        // Called at 50hz.
    }
}
