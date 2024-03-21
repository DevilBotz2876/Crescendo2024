package frc.robot.subsystems.led;

import org.littletonrobotics.junction.AutoLog;

public interface LedIO {
  @AutoLog
  public static class LedIOInputs {
    public int red = 0;
    public int green = 0;
    public int blue = 0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(LedIOInputs inputs) {}

  public default void setColor(int red, int green, int blue) {}
}
