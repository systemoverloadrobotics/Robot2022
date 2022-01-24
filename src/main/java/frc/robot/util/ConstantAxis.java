package frc.robot.util;

import java.util.function.DoubleSupplier;
import frc.robot.util.LogitechExtremePro3DJoystick.AxisType;

public class ConstantAxis {
  private final int stick;
  private final AxisType type;
  
  public ConstantAxis(int joystick, AxisType type) {
    this.stick = joystick;
    this.type = type;
  }

  public DoubleSupplier supplier() {
    return () -> ConstantInput.get().lazyJoy(stick).getAxis(type);
  }

  public double get() {
    return ConstantInput.get().lazyJoy(stick).getAxis(type);
  }
}
