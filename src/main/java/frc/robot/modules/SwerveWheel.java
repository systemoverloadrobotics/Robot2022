package frc.robot.modules;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import frc.robot.util.Utils;


public class SwerveWheel {
    private TalonFX speed;
    private TalonSRX steer;

    public SwerveWheel(int speedInt, int rotationInt) {
        speed = new TalonFX(speedInt); 
        steer = new TalonSRX(rotationInt);
        
        speed.configFactoryDefault();
        steer.configFactoryDefault();

        steer.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        
    }

    public void setSpeed(double velocity) {
        speed.set(ControlMode.Velocity, velocity);
    }

    public void setRotation(double velocity) {
        steer.set(ControlMode.Position, Utils.tickToDegrees(velocity, 4096));
    }

    public double getSpeed() {
	    return speed.getSelectedSensorVelocity(); 
    }

    public double getRotation() {
	    return Utils.tickToDegrees(steer.getSelectedSensorPosition(), 4096); 
    }

    public void periodic(){
        //
    }


}
