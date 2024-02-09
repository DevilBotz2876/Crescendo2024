package frc.robot.subsystems.shooter;

public interface Shooter {
  /**
   * Set the voltage of motors for the shooter.
   *
   * @param volts The volts to set. Value should be between -12.0 and 12.0.
   */
  public default void runVoltage(double volts) {}

  /** Run closed loop at the specified velocity */
  public default void runVelocity(double velocityRPM) {}

  public double getCurrentSpeed();

  public double getVoltage();
}
