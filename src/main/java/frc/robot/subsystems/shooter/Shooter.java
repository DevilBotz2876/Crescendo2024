package frc.robot.subsystems.shooter;

public interface Shooter {
  /** Disable the voltage of a motors for the shooter. */
  public default void disable() {}

  /** Enable the voltage of a motors for the shooter. */
  public default void enable() {}

  /**
   * Set the voltage of motors for the shooter.
   *
   * @param volts The volts to set. Value should be between -12.0 and 12.0.
   */
  public default void setVoltage(double volts) {}

  public double getCurrentSpeed();

  public double getVoltage();
}
