package frc.robot.subsystems;

public interface Intake {
  // Enables motors so it feeds into intake
  public default void noteIn() {}

  // Enables motors so it feeds out of intake
  public default void noteOut() {}

  // Disable the motors for the Intake
  public default void disable() {}
}
