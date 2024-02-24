package frc.robot.subsystems.vision;

public interface Vision {
  public default double getDistanceToAprilTag(int aprilTagId) {
    return -1;
  }
}
