package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.core.SwerveWheel;


public class SwerveDrive {
    private SwerveWheel topLeftWheel = new SwerveWheel(0, 1);
    private SwerveWheel topRightWheel = new SwerveWheel(2, 3);
    private SwerveWheel bottomLeftWheel = new SwerveWheel(4, 5);
    private SwerveWheel bottomRightWheel = new SwerveWheel(6, 7);

    private PigeonIMU gyro = new PigeonIMU(0);
    
    enum WheelPosition { topLeft, topRight, bottomLeft, bottomRight };

    public SwerveDrive() {

    }

    public void steer(double x1, double y1, double speed) {
        double angle = Math.tan(y1 / x1);
        
        // TEMP
        setWheel(WheelPosition.bottomLeft, speed, angle * 360);
        setWheel(WheelPosition.bottomRight, speed, angle * 360);
        setWheel(WheelPosition.topLeft, speed, angle * 360);
        setWheel(WheelPosition.topRight, speed, angle * 360);
    }

    public void setWheel(WheelPosition pos, double speed, double angle) {
        switch (pos) {
            case topLeft:
                System.out.print("yay");
                topLeftWheel.setSpeed(speed);
                topLeftWheel.setRotation(angle);
                break;
            case topRight:
                System.out.print("yayy");
                topRightWheel.setSpeed(speed);
                topRightWheel.setRotation(angle);
                break;
            case bottomLeft:
                System.out.print("yayyy");
                bottomLeftWheel.setSpeed(speed);
                bottomLeftWheel.setRotation(angle);
                break;
            case bottomRight:
                System.out.println("yayyyy");
                bottomRightWheel.setSpeed(speed);
                bottomRightWheel.setRotation(angle);
                break; 
        }
    }

}