package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
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

public class VisionSubsystem extends SubsystemBase implements Vision {
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

  PhotonPipelineResult result;
  @AutoLogOutput int targetsVisible;
  int speakerId;
  @AutoLogOutput double speakerDistance;
  @AutoLogOutput double speakerYaw;
  int ampId;
  @AutoLogOutput double ampDistance;
  @AutoLogOutput double ampYaw;
  @AutoLogOutput Pose2d estimatedPose;
  double estimatedPoseTimestamp;

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
    targetsVisible = result.targets.size();

    Optional<EstimatedRobotPose> currentEstimatedRobotPose = poseEstimator.update();
    if (currentEstimatedRobotPose.isPresent()) {
      estimatedPose = currentEstimatedRobotPose.get().estimatedPose.toPose2d();
      estimatedPoseTimestamp = currentEstimatedRobotPose.get().timestampSeconds;
    }

    if (DriverStation.getAlliance().isPresent()
        && (DriverStation.getAlliance().get() == DriverStation.Alliance.Red)) {
      speakerId = 4; // Red Speaker Center
      ampId = 5; // Red Amp
    } else {
      speakerId = 7; // Blue Speaker Center
      ampId = 6; // Blue Amp
    }

    Optional<Double> distance;
    Optional<Double> yaw;
    distance = getDistanceToAprilTag(speakerId);
    yaw = getYawToAprilTag(speakerId);
    if (distance.isPresent()) {
      speakerDistance = distance.get();
    }
    if (yaw.isPresent()) {
      speakerYaw = yaw.get();
    }

    distance = getDistanceToAprilTag(ampId);
    yaw = getYawToAprilTag(ampId);
    if (distance.isPresent()) {
      ampDistance = distance.get();
    }
    if (yaw.isPresent()) {
      ampYaw = yaw.get();
    }
  }

  private PhotonTrackedTarget findAprilTag(int id) {
    for (PhotonTrackedTarget target : result.targets) {
      if (target.getFiducialId() == id) {
        return target;
      }
    }
    return null;
  }

  @Override
  public Optional<Double> getDistanceToAprilTag(int id) {
    PhotonTrackedTarget target = findAprilTag(id);

    if (target != null) {
      return Optional.of(
          PhotonUtils.calculateDistanceToTargetMeters(
              robotToCamera.getZ(),
              fieldLayout.getTagPose(target.getFiducialId()).get().getZ(),
              -robotToCamera.getRotation().getY(),
              Units.degreesToRadians(target.getPitch())));
    }
    return Optional.empty();
  }

  @Override
  public Optional<Double> getYawToBestTarget() {
    if (result.hasTargets()) {
      return Optional.of(result.getBestTarget().getYaw());
    }
    return Optional.empty();
  }

  @Override
  public Optional<Double> getYawToAprilTag(int id) {
    PhotonTrackedTarget target = findAprilTag(id);

    if (target != null) {
      return Optional.of(target.getYaw());
    }

    return Optional.empty();
  }
  /*
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
  */
}
