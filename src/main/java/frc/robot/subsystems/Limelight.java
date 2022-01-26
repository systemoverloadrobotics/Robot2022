package frc.robot.subsystems;

import java.util.function.Supplier;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Limelight extends SubsystemBase {

    NetworkTable networkTable;

    public Limelight() {
        networkTable = NetworkTableInstance.getDefault().getTable("Limelight");
    }
    
    public Supplier<Object> valueSupplier(LimelightTable e) {
        return () -> networkTable.getEntry(e.key).getValue().getValue();
    }
    public <T> Supplier<T> valueSupplier(LimelightTable e, Class<T> clazz) {
        return () -> clazz.cast(networkTable.getEntry(e.key).getValue().getValue());
    }

    /** @return If the limelight is connected */
    public boolean connected() {
        return networkTable.getEntry("tx").exists();
    }

    /** @return If the limelight has found a target */
    public boolean canSeeTarget() {
        return valueSupplier(LimelightTable.TV, Integer.class).get().intValue() > 0;
    }


    // returns the size of the target
    public double getTargetArea() {
        return valueSupplier(LimelightTable.TA, Double.class).get().doubleValue();
    }

    public static enum LimelightTable {
        TV("tv"), 
        TX("tx"), 
        TY("ty"), 
        TA("ta"), 
        TS("ts"), 
        TL("tl"), 
        TSHORT("tshort"), 
        TLONG("tlong"), 
        THOR("thor"), 
        TVERT("tvert"), 
        GETPIPE("getpipe"), 
        CAMTRAN("camtran"), 
        TC("tc");

        String key;
        
        LimelightTable(String key) {
            this.key = key;
        }
    }
}
