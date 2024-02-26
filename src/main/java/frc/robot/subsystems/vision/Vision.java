package frc.robot.subsystems.vision;

import java.util.Optional;

public interface Vision {
  /**
   * Returns the yaw in degrees to the best target
   *
   * @return yaw to the best target (in degrees)
   */
  public Optional<Double> getYawToBestTarget();

  /**
   * Returns the distance to the specified april tag in meters
   *
   * @param id AprilTag ID
   * @return distance to the specified april tag (in meters).
   */
  public Optional<Double> getDistanceToAprilTag(int id);

  /**
   * Returns the yaw in degrees to th specified april tag in meters
   *
   * @param id AprilTag ID
   * @return yaw to the specified april tag (in degrees).
   */
  public Optional<Double> getYawToAprilTag(int id);
}
