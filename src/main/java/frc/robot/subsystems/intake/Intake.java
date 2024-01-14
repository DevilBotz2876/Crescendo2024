package frc.robot.subsystems.intake;

public interface Intake {
   /**
   * Disable the speed of the motor for the intake.
   */
  public default void disable() {}

  /**
   * Enable the speed of the motor for the intake.
   */
  public default void enable() {}

  /**
   * Set the speed of the motor for the intake.
   *
   * @param speed The speed to set. Value should be between -1.0 and 1.0.
   */
  public default void setSpeed(double speed) {}
}
