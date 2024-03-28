package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import java.text.DecimalFormat;
import java.util.Optional;
import java.util.function.Supplier;

public interface Vision {
  public class VisionPose {
    public static DecimalFormat doubleFormat = new DecimalFormat("0.00");
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
      return "timestamp:"
          + VisionPose.doubleFormat.format(timestamp)
          + " cameraName:"
          + cameraName
          + " pose2d:(x:"
          + VisionPose.doubleFormat.format(robotPose.getX())
          + " y: "
          + VisionPose.doubleFormat.format(robotPose.getY())
          + " yaw: "
          + VisionPose.doubleFormat.format(robotPose.getRotation().getDegrees())
          + ")";
    }
  }

  public default boolean setPrimaryCamera(String name) {
    return true;
  }

  public Optional<Integer> getBestTargetId();

  public Optional<Double> getDistanceToBestTarget();

  /**
   * Returns the yaw in degrees to the best target (relative to the primary camera)
   *
   * @return yaw to the best target (in degrees)
   */
  public Optional<Double> getYawToBestTarget();

  /**
   * Returns the distance to the specified april tag in meters (relative to the primary camera)
   *
   * @param id AprilTag ID
   * @return distance to the specified april tag (in meters).
   */
  public Optional<Double> getDistanceToAprilTag(int id);

  /**
   * Returns the yaw in degrees to the specified april tag in meters (relative to the primary
   * camera)
   *
   * @param id AprilTag ID
   * @return yaw to the specified april tag (in degrees).
   */
  public Optional<Double> getYawToAprilTag(int id);

  public default void enableSimulation(Supplier<Pose2d> poseSupplier, boolean enableWireFrame) {}
}
