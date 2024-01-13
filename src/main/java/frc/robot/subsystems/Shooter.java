package frc.robot.subsystems;

public interface Shooter {
  // Disable the shooter
  public default void disable(double voltage) {}

  // Enable the shooter.
  public default void enable(double voltage) {}

  // Set the voltage of the motors.
  public default double setVoltage () {
    return 0;
  }
}
