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

    private boolean target;
    private double x;
    private double y;
    private double area;

    public Limelight() {

        networkTable = NetworkTableInstance.getDefault().getTable("Limelight");

        targetEntry = networkTable.getEntry("tv");// Whether the limelight has any valid targets (0 or 1)
        horizontalAngleOffSet = networkTable.getEntry("tx"); // Horizontal Offset From Crosshair To Target (-27 degrees
                                                             // to 27 degrees)
        verticalAngleOffSet = networkTable.getEntry("ty"); // Vertical Offset From Crosshair To Target (-20.5 degrees to
                                                           // 20.5 degrees)
        ta = networkTable.getEntry("ta"); // Target Area (0% of image to 100% of image

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

    /** @return If the limelight has found a target */
    public boolean canSeeTarget() {
        return targetEntry.getNumber(0).intValue() > 0;
    }


    // returns the size of the target
    public double getTargetArea() {
        return ta.getNumber(0).doubleValue();
    }


    public void periodic() {

        target = targetEntry.getBoolean(false);
        x = horizontalAngleOffSet.getDouble(0.0);
        y = verticalAngleOffSet.getDouble(0.0);
        area = ta.getDouble(0.0);

    }

}
