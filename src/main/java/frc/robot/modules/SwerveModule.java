package frc.robot.modules;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.CANCoder;

// SwerveModule manages an individual swerve drive module on the robot.
public class SwerveModule {
    // Should contain the two motor controllers, tracking, methods for setting,
    // and encoder output.

    public SwerveModule(TalonFX powerController, TalonSRX steerController, CANCoder absSensor) {

    }

    public void periodic() {
        // Called at 50hz.
    }
}
