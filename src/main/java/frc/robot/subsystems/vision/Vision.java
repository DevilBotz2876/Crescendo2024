package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import java.util.List;
import java.util.Optional;

public interface Vision {
  public class VisionPose {
    public String cameraName;
    public Pose2d robotPose;
    public double timestamp;

    //    Matrix<N3,N1> visionMeasurementStdDevs; // Vision measurement standard deviation that will
    // be sent to the SwerveDrivePoseEstimator.The standard deviation of the vision measurement, for
    // best accuracy calculate the standard deviation at 2 or more points and fit a line to it with
    // the calculated optimal standard deviation. (Units should be meters per pixel). By optimizing
    // this you can get * vision accurate to inches instead of feet.

    /**
     * @param robotPose // Robot Pose2d as measured by vision
     * @param timestamp // Timestamp the measurement was taken as time since startup, should be
     *     taken from Timer.getFPGATimestamp() or similar sources.
     * @param cameraName // Name of camera this pose was calculated from
     */
    public VisionPose(Pose2d robotPose, double timestamp, String cameraName) {
      this.robotPose = robotPose;
      this.timestamp = timestamp;
      this.cameraName = cameraName;
    }

    public VisionPose() {
      this(new Pose2d(), -1, "");
    }

    @Override
    public String toString() {
      return "timestamp:" + timestamp + " cameraName:" + cameraName + " pose2d:" + robotPose;
    }
  }

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

  /**
   * Returns a list of vision-based estimated poses
   *
   * @return list of estimated poses. list length may be zero if updated pose estimation data is not
   *     available.
   */
  public List<VisionPose> getEstimatedPose();
}
