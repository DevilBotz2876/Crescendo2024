package frc.robot.subsystems;

public interface Shooter {
  // Disable the shooter
  public default void disable() {}

  // Enable the shooter with the .
  public default void noteOut(double voltage) {}

  // Set the voltage of the motors.
  public default double setVoltage() {
    return 0;
  }
}
