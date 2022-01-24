package frc.robot.util;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.wpilibj.GenericHID;

/** Edited form of Joystick.class */

public class LogitechExtremePro3DJoystick extends GenericHID {
  public static final byte DEFAULT_X_CHANNEL = 0;
  public static final byte DEFAULT_Y_CHANNEL = 1;
  public static final byte DEFAULT_TWIST_CHANNEL = 2;
  public static final byte DEFAULT_THROTTLE_CHANNEL = 3;
  public static final byte DEFAULT_HAT_CHANNEL = 4;

  public enum AxisType {
    X(0, DEFAULT_X_CHANNEL),
    Y(1, DEFAULT_Y_CHANNEL),
    TWIST(2, DEFAULT_TWIST_CHANNEL),
    THROTTLE(3, DEFAULT_THROTTLE_CHANNEL),
    HAT(4, DEFAULT_HAT_CHANNEL);

    public final int value;
    private int channel;

    AxisType(int value, int channel) {
      this.value = value;
      this.channel = channel;
    }

    public int getChannel() {
      return channel;
    }
  }

  public enum ButtonType {
    TRIGGER(1),
    THUMB_BUTTON(2),
    B3(3),
    B4(4),
    B5(5),
    B6(6),
    B7(7),
    B8(8),
    B9(9),
    B10(10),
    B11(11),
    B12(12);
    
    public final int value;

    ButtonType(int value) {
      this.value = value;
    }
  }

  /**
   * Construct an instance of a LogitechExtremePro3DJoystick.
   *
   * @param port The port index on the Driver Station that the joystick is plugged into.
   */
  public LogitechExtremePro3DJoystick(final int port) {
    super(port);

    HAL.report(tResourceType.kResourceType_Joystick, port + 1);
  }

  /**
   * Get the ConstantAxis associated with an axis.
   *
   * @param type The axis which is being read.
   */
  public ConstantAxis getConstantAxis(AxisType type) {
    return new ConstantAxis(getPort(), type);
  }

  /**
   * Get the ConstantButton associated with a button.
   *
   * @param type The axis which is being read.
   */
  public ConstantButton getConstantButton(ButtonType type) {
    return new ConstantButton(getPort(), type);
  }

  /**
   * Set the channel associated with an axis.
   *
   * @param axis The axis which is being configured.
   * @param channel The channel to set the axis to.
   */
  public void setChannel(AxisType axis, int channel) {
    axis.channel = (byte) channel;
  }

  /**
   * Get the channel associated with an axis.
   *
   * @param axis The axis which is being read.
   */
  public int getChannel(AxisType axis) {
    return axis.channel;
  }

  /**
   * Get the axis value on the joystick. This depends on the mapping of the joystick connected to the
   * current port.
   *
   * @param axis The axis which is being accessed.
   * @return The axis value of the joystick.
   */
  public final double getAxis(AxisType axis) {
    return getRawAxis(axis.channel);
  }

  /**
   * Read the state of a button on the joystick.
   *
   * @param button The button which is being accessed.
   * @return The state of the button.
   */
  public boolean getButtonState(ButtonType button) {
    return getRawButton(button.value);
  }

  /**
   * Whether the button was pressed since the last check.
   *
   * @param button The button which is being accessed.
   * @return Whether the button was pressed since the last check.
   */
  public boolean getButtonPressed(ButtonType button) {
    return getRawButtonPressed(button.value);
  }

  /**
   * Whether the button was released since the last check.
   *
   * @param button The button which is being accessed.
   * @return Whether the button was released since the last check.
   */
  public boolean getButtonReleased(ButtonType button) {
    return getRawButtonReleased(button.value);
  }

  /**
   * Get the magnitude of the direction vector formed by the joystick's current position relative to
   * its origin.
   *
   * @return The magnitude of the direction vector
   */
  public double getMagnitude() {
    return Math.sqrt(Math.pow(getAxis(AxisType.X), 2) + Math.pow(getAxis(AxisType.Y), 2));
  }

  /**
   * Get the direction of the vector formed by the joystick and its origin in radians.
   *
   * @return The direction of the vector in radians
   */
  public double getDirectionRadians() {
    return Math.atan2(getAxis(AxisType.X), -getAxis(AxisType.Y));
  }

  /**
   * Get the direction of the vector formed by the joystick and its origin in degrees.
   *
   * @return The direction of the vector in degrees
   */
  public double getDirectionDegrees() {
    return Math.toDegrees(getDirectionRadians());
  }
}
