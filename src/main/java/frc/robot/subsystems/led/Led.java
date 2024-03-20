package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj2.command.Command;

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

  public Command getNoteDetectionCommand();

  public Command getAmpModeCommand();

  public Command getSpeakerModeCommand();
}
