package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.sensors.PigeonIMU;

import frc.robot.Constants;
import frc.robot.core.SwerveWheel;

public class SwerveDrive {
    private SwerveWheel topLeftWheel = new SwerveWheel(0, 1);
    private SwerveWheel topRightWheel = new SwerveWheel(2, 3);
    private SwerveWheel bottomLeftWheel = new SwerveWheel(4, 5);
    private SwerveWheel bottomRightWheel = new SwerveWheel(6, 7);

    private PigeonIMU gyro = new PigeonIMU(0);

    enum WheelPosition {
        topLeft, topRight, bottomLeft, bottomRight
    };

    public SwerveDrive() {

    }

    public void steer(double x1, double y1, double x2, double speed) {

        double temp = (-y1 * Math.cos(gyro.getYaw())) + (x1 * Math.sin(gyro.getYaw()));
        x1 = (-y1 * Math.sin(gyro.getYaw())) + (x1 * Math.cos(gyro.getYaw()));
        y1 = -temp;

        double r = Math
                .sqrt(Math.pow(Constants.RobotDimensions.LENGTH, 2) + Math.pow(Constants.RobotDimensions.WIDTH, 2)) / 2;

        // STR = x1, FWD = -y1
        double a = x1 - (x2 * (Constants.RobotDimensions.LENGTH / r));
        double b = x1 + (x2 * (Constants.RobotDimensions.LENGTH / r)); 
        double c = -y1 - (x2 * (Constants.RobotDimensions.WIDTH / r));
        double d = -y1 + (x2 * (Constants.RobotDimensions.WIDTH / r));

        //speed
        double ws1 = Math.sqrt(Math.pow(b, 2) + Math.pow(c, 2));
        double ws2 = Math.sqrt(Math.pow(b, 2) + Math.pow(d, 2));
        double ws3 = Math.sqrt(Math.pow(a, 2) + Math.pow(d, 2));
        double ws4 = Math.sqrt(Math.pow(a, 2) + Math.pow(c, 2));

        double max = ws1;
        if(ws2 > max) max = ws2;
        if(ws3 > max) max = ws3;
        if(ws4 > max) max = ws4;
        if(max > 1){
            ws1 /= max;
            ws2 /= max;
            ws3 /= max;
            ws4 /= max;
        }

        // TEMP
        setWheel(WheelPosition.bottomLeft, ws3, (Math.atan2(b,c) * (180/Math.PI)));
        setWheel(WheelPosition.bottomRight, ws4, (Math.atan2(b,d) * (180/ Math.PI)));
        setWheel(WheelPosition.topLeft, ws2, (Math.atan2(a,d) * (180/Math.PI)));
        setWheel(WheelPosition.topRight, ws1, (Math.atan2(a, c) * (180/Math.PI)));
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