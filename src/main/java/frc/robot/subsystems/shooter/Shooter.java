package frc.robot.subsystems.shooter;

public interface Shooter {
  // Disable the shooter
  public default void disable() {}

  // Enable the shooter with the .
  public default void enable() {}

  // Set the speed of the motors.
  public default void setSpeed(double speed) {}
}
