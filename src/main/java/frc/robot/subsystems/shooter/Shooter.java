package frc.robot.subsystems.shooter;

public interface Shooter {
  /**
   * Disable the speed of a motors for the shooter.
   */
  public default void disable() {}

  /**
   * Enable the speed of a motors for the shooter.
   */
  public default void enable() {}

  /**
   * Set the speed of motors for the shooter.
   *
   * @param speed The speed to set. Value should be between -1.0 and 1.0.
   */
  public default void setSpeed(double speed) {}
}
