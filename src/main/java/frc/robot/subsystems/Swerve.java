package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.modules.SwerveModule;

public class Swerve extends SubsystemBase {

  public static enum Corner {
    FRONT_LEFT,
    FRONT_RIGHT,
    BACK_LEFT,
    BACK_RIGHT,
  }

  private Map<Corner, SwerveModule> modules = new HashMap<>();

  public Swerve() {
    // Create four modules with correct controllers, add to modules
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    for (var module : modules.values()) {
      module.periodic();
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

}
