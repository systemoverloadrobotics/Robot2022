package frc.robot.util;

import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.util.LogitechExtremePro3DJoystick.ButtonType;

public class ConstantButton {
  private final int joystick;
  private final ButtonType type;  

  public ConstantButton(int joystick, ButtonType type) {
    this.joystick = joystick;
    this.type = type;
  }

  public JoystickButton get() {
    return new JoystickButton(ConstantInput.get().lazyJoy(joystick), type.value);
  }

  public BooleanSupplier state() {
    return () -> ConstantInput.get().lazyJoy(joystick).getButtonState(type);
  }
}
