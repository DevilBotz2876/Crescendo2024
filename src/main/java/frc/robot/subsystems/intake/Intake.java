package frc.robot.subsystems.intake;

public interface Intake {
  public default boolean isPieceDetected(boolean intakePieceDetection) {
    return false;
  }

  public default void runVoltage(double volts) {}

  public default boolean isPieceShooterDetected() {
    return false;
  }

}
