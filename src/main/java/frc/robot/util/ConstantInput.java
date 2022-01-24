package frc.robot.util;

import java.util.HashMap;
import java.util.Map;

// ConstantInput is a singleton that manages the Joystick inputs to the program,
// allowing for "constant" declarations of Joystick buttons and axes.
public class ConstantInput {
  private static final ConstantInput inst = new ConstantInput();  

  private final Map<Integer, LogitechExtremePro3DJoystick> joys = new HashMap<>();

  public static ConstantInput get() {
    return inst;
  }

  public LogitechExtremePro3DJoystick lazyJoy(int idx) {
    return joys.putIfAbsent(idx, new LogitechExtremePro3DJoystick(idx));
  }
}
