package frc.robot.core;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;


public class SwerveWheel {
    private WPI_VictorSPX speed;
    private WPI_TalonFX rotation;

    public SwerveWheel(int speedInt, int rotationInt) {
        speed = new WPI_VictorSPX(speedInt); 
        rotation = new WPI_TalonFX(rotationInt);        
    }

    public void setSpeed(double velocity) {
        speed.set(ControlMode.Velocity, velocity);
    }

    public void setRotation(double velocity) {
        rotation.set(ControlMode.Position, Utils.degreesToTicks(velocity));
    }

    public double getSpeed() {
	    return speed.getSelectedSensorVelocity(); 
    }

    public double getRotation() {
	    return Utils.degreesToTicks(rotation.getSelectedSensorPosition()); 
    }
}
