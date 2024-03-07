package frc.robot.subsystems.intake;

public interface Intake {
  public default boolean isPieceDetected() {
    return false;
  }

  public default void runVoltage(double volts) {}
}
