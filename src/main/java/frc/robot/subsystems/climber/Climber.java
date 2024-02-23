package frc.robot.subsystems.climber;

public interface Climber {
  /** Extends climber arms min limit */
  public void extend();

  /** Retracts climber to min limit */
  public void retract();

  public void runVoltage(double volts);

  public default void runVoltageLeft(double volts) {}

  public default void runVoltageRight(double volts) {}

  public void resetPosition();

  public default void autoZeroMode(boolean enable) {}
}
