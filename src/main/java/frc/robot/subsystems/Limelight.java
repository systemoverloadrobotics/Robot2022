package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import static java.lang.Math.*;
import static frc.robot.util.Utils.*;

public class Limelight extends SubsystemBase {

    NetworkTable networkTable;
    private NetworkTableEntry targetEntry;
    private NetworkTableEntry horizontalAngleOffSet;
    private NetworkTableEntry verticalAngleOffSet;
    private NetworkTableEntry ta;

    public Limelight() {

        networkTable = NetworkTableInstance.getDefault().getTable("Limelight");
        // Horizontal Offset From Crosshair To Target (-27 degrees to 27 degrees)
        horizontalAngleOffSet = networkTable.getEntry("tx");
        // Vertical Offset From Crosshair To Target (-20.5 degrees to 20.5 degrees)
        verticalAngleOffSet = networkTable.getEntry("ty"); 
        // Target Area (0% of image to 100% of image
        ta = networkTable.getEntry("ta"); 

    }

    /**
     * @return a horizontal angle from -27 to 27 between the target and the camera
     */
    public double getHorizontalAngle() {
        return horizontalAngleOffSet.getDouble(0);
    }

    /** @return a vertical angle from -27 to 27 between the target and the camera */
    public double getVerticalAngle() {
        return verticalAngleOffSet.getDouble(0);
    }

    /** @return If the limelight is connected */
    public boolean connected() {
        return networkTable.getEntry("tx").exists();
    }

    // returns the size of the target
    public double getTargetArea() {
        return ta.getNumber(0).doubleValue();
    }

    // returns distance in meters
    public double getDistance() {
        final double a2 = getVerticalAngle();
        return (Constants.HUB_HEIGHT - Constants.LIMELIGHT_HEIGHT) / Math.tan(Constants.LIMELIGHT_ANGLE + a2)        ;
    }

    public double getVelocity() {
        final double xpos = getDistance();
        final double ypos = Constants.LIMELIGHT_HEIGHT;
        final double launchAngle = Math.toRadians(Constants.LIMELIGHT_ANGLE);
        final double hub = Constants.HUB_HEIGHT;
        /*
            The stuff below is very interesting, it was calculated by using a System of Equations.
            We know the target xpos, the target ypos, the launch angle, and the hub.
            We need to find the launch speed and the time.
            We are given 2 equations to do this, the x value finder and the y value finder.
            We can plug in filler variables into the projectile motion equations and solve for launch speed.
            Then, we can substitute the launch speed in one equation with the other equation launch speed.
            We can use this new equation to solve for time.
            Once we have solved for time, we can plug it into either the x value finder or the y value finder.
            This gives us the target launch speed in meters/second.
        */
        final double time = sqrt((xpos * tan(launchAngle) - 2.64 + ypos) / hub);
        final double launchSpeed = (xpos * sec(launchAngle)) / time;

        return launchSpeed;
    }
}
