package frc.robot.subsystems.vision;

public interface Vision {

  public default boolean hasTarget() {
    return false;
  }

  public default double getRange() {
    return 0.0;
  }

  public default double getYaw() {
    return 0.0;
  }
}
