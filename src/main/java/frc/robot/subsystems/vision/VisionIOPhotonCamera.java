package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Transform3d;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class VisionIOPhotonCamera implements VisionIO {
  private final PhotonCamera camera;
  private final Transform3d robotToCamera;
  private final PhotonPoseEstimator poseEstimator;

  /**
   * Location of the camera view relative to the robot (in meters)
   *
   * <p>Translation(x,y,z) - location of the camera relative to the center of the robot x +/- is
   * forward/back from center y +/- is right/left from center. z is relative to the ground (and
   * should be positive) E.g. Translation(0.5,0,0.5) is 1/2 meter forward of center and 1/2 meter
   * above center Rotation(roll, pitch, yaw) - orientation of the camera view roll +/- is CCW/CW
   * around x-axis (should generally be 0, 90, 180 or 270) pitch +/- is up/down around y-axis
   * (should generally be >= 0) yaw +/- is left/right around the z-axis E.g. Rotation(0,0,0) is
   * facing straight forward
   */
  public VisionIOPhotonCamera(
      String cameraName, Transform3d robotToCamera, AprilTagFieldLayout aprilTagFieldLayout) {
    this.camera = new PhotonCamera(cameraName);
    this.robotToCamera = robotToCamera;
    if (robotToCamera != null) {
      this.poseEstimator =
          new PhotonPoseEstimator(
              aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCamera);
      this.poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    } else {
      this.poseEstimator = null;
    }
  }

  public VisionIOPhotonCamera(String cameraName) {
    this(cameraName, null, null);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.targets = camera.getLatestResult().getTargets().toArray();
    if (poseEstimator != null) {
      inputs.estimatedRobotPose = poseEstimator.update().get().estimatedPose;
    }
  }

  @Override
  public Transform3d getRobotToCameraPose() {
    return robotToCamera;
  }
}
