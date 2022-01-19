package frc.robot.modules;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.CANCoder;
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
        steerController.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    }

    public void setSteerRotation(double angle){
        steerController.set(ControlMode.Position, Utils.degreesToTicks(angle, 2048));
    }

    public double getSteerPosition(){
        return Utils.ticksToDegrees(steerController.getSelectedSensorPosition(), 2048);
    }

    public void setVelocity(double velocity){
        powerController.set(ControlMode.Velocity, velocity);
    }

    public double getVelocity(){
        return powerController.getSelectedSensorVelocity();
    }
  
    public void periodic() {
        // Called at 50hz.
    }
}
