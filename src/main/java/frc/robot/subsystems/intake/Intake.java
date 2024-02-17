package frc.robot.subsystems.intake;

public interface Intake {
  /** Disable the speed of the motor for the intake. */
  public default void disable() {}

  public default boolean isPieceDetected(boolean intakePieceDetection) {
    return false;
  }

  /** Enable the speed of the motor for the intake. */
  public default void enable() {}

  public default void setVoltage(double volts) {}
}
