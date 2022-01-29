package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

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
}
