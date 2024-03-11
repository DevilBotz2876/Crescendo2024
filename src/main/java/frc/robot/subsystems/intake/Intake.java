package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.config.RobotConfig.IntakeConstants;

public interface Intake {
  public default boolean isPieceDetected() {
    return false;
  }

  public default void runVoltage(double volts) {}

  public default double getCurrentVoltage() {
    return 0;
  }

  public default void turnOn() {
    runVoltage(IntakeConstants.defaultSpeedInVolts);
  }

  public default void turnOff() {
    runVoltage(0);
  }

  public Command getTurnOnCommand();

  public Command getTurnOffCommand();
}
