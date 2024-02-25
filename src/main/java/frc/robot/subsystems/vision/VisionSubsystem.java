package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Optional;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionSubsystem extends SubsystemBase {
  private final PhotonCamera camera;

  /*
   * Location of the camera view relative to the robot (in meters)
   *
   * Translation(x,y,z) - location of the camera relative to the center of the robot (in meters)
   *    x +/- is forward/back from center
   *    y +/- is right/left from center.
   *    z is relative to the ground (and should be positive)
   *  E.g. Translation(0.5,0,0.5) is 1/2 meter forward of center and 1/2 meter above center
   *
   * Rotation(roll, pitch, yaw) - orientation of the camera view (in radians)
   *    roll +/- is CCW/CW around x-axis (should generally be 0)
   *    pitch +/- is up/down around y-axis (should generally be >= 0)
   *    yaw +/- is left/right around the z-axis
   *  E.g. Rotation(0,0,0) is facing straight forward
   */
  private final Transform3d robotToCamera;
  private final AprilTagFieldLayout fieldLayout;
  private final PhotonPoseEstimator poseEstimator;

  private final PhotonCameraSim cameraSim;
  private final VisionSystemSim visionSim;
  private final Supplier<Pose2d> poseSupplier;

  @AutoLogOutput PhotonPipelineResult result;
  @AutoLogOutput int targetId;
  @AutoLogOutput Pose3d targetPose;
  @AutoLogOutput double speakerDistance;
  @AutoLogOutput double speakerYaw;
  @AutoLogOutput Pose2d estimatedPose;
  @AutoLogOutput double estimatedPoseTimestamp;

  public VisionSubsystem(Supplier<Pose2d> poseSupplier) {
    camera = new PhotonCamera("photonvision");
    robotToCamera =
        new Transform3d(
            new Translation3d(0.221, 0, .164), new Rotation3d(0, Units.degreesToRadians(-20), 0));
    fieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    poseEstimator =
        new PhotonPoseEstimator(
            this.fieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            this.camera,
            this.robotToCamera);

    cameraSim = new PhotonCameraSim(camera);
    cameraSim.enableDrawWireframe(true); // Could be Processor intensive!
    visionSim = new VisionSystemSim("main");
    visionSim.addAprilTags(this.fieldLayout);
    visionSim.addCamera(cameraSim, robotToCamera);
    this.poseSupplier = poseSupplier;
  }

  @Override
  public void periodic() {
    if (visionSim != null) {
      visionSim.update(poseSupplier.get());
    }

    result = camera.getLatestResult();

    Optional<EstimatedRobotPose> currentEstimatedRobotPose = poseEstimator.update();
    if (currentEstimatedRobotPose.isPresent()) {
      estimatedPose = currentEstimatedRobotPose.get().estimatedPose.toPose2d();
      estimatedPoseTimestamp = currentEstimatedRobotPose.get().timestampSeconds;
    }

    if (DriverStation.getAlliance().isPresent()
        && (DriverStation.getAlliance().get() == DriverStation.Alliance.Red)) {
      targetId = 4; // Red Speaker Center
    } else {
      targetId = 7; // Blue Speaker Center
    }

    speakerDistance = getDistanceToAprilTag(targetId);
    speakerYaw = getDistanceToAprilTag(targetId);
    targetPose = getPose3dToAprilTag(targetId);
  }

  private PhotonTrackedTarget findAprilTag(int id) {
    for (PhotonTrackedTarget target : result.targets) {
      if (target.getFiducialId() == id) {
        return target;
      }
    }
    return null;
  }

  double getDistanceToAprilTag(int id) {
    PhotonTrackedTarget target = findAprilTag(id);

    if (target != null) {
      return PhotonUtils.calculateDistanceToTargetMeters(
          robotToCamera.getZ(),
          fieldLayout.getTagPose(target.getFiducialId()).get().getZ(),
          -robotToCamera.getRotation().getY(),
          Units.degreesToRadians(target.getPitch()));
    }
    return -1;
  }

  public double getYawToAprilTag(int id) {
    PhotonTrackedTarget target = findAprilTag(id);

    if (target != null) {
      return target.getYaw();
    }

    return -1;
  }

  public Pose3d getPose3dToAprilTag(int id) {
    PhotonTrackedTarget target = findAprilTag(id);
    if (target != null) {
      return PhotonUtils.estimateFieldToRobotAprilTag(
          target.getBestCameraToTarget(),
          fieldLayout.getTagPose(target.getFiducialId()).get(),
          robotToCamera);
    }
    return null;
  }
}
