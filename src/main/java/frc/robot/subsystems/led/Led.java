package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;

public interface Led {
  public default void setColor(int red, int green, int bluw) {}

  public default int getRed() {
    return 0;
  }

  public default int getGreen() {
    return 0;
  }

  public default int getBlue() {
    return 0;
  }

  public default void add2dSim(Mechanism2d mech2d) {}
}
