package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.RobotState;
import java.util.ArrayList;
import java.util.List;
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
  private class VisionCameraImpl {
    private static int numCameras = 0;
    private final PhotonCamera camera;
    private final Transform3d robotToCamera;
    private final PhotonPoseEstimator poseEstimator;
    private final int index;
    private PhotonCameraSim simCamera;
    private PhotonPipelineResult result;
    private Optional<EstimatedRobotPose> estimatedRobotPose;

    private VisionCameraImpl(VisionCamera camera, AprilTagFieldLayout fieldLayout) {
      this.camera = new PhotonCamera(camera.name);
      this.robotToCamera = camera.robotToCamera;
      this.poseEstimator =
          new PhotonPoseEstimator(
              fieldLayout,
              PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
              this.camera,
              this.robotToCamera);
      this.index = numCameras++;
    }

    private void update() {
      result = camera.getLatestResult();
      estimatedRobotPose = poseEstimator.update();
    }

    private double getLatestTimestamp() {
      return result.getTimestampSeconds();
    }

    private boolean hasTargets() {
      return result.hasTargets();
    }

    private List<PhotonTrackedTarget> getTargets() {
      return result.targets;
    }

    private PhotonTrackedTarget getBestTarget() {
      return result.getBestTarget();
    }

    private Optional<EstimatedRobotPose> getEstimatedRobotPose() {
      return estimatedRobotPose;
    }

    private int getIndex() {
      return index;
    }

    private String getName() {
      return camera.getName();
    }

    private Transform3d getRobotToCamera() {
      return robotToCamera;
    }
  }

  private final List<VisionCameraImpl> cameras = new ArrayList<VisionCameraImpl>();
  private VisionCameraImpl primaryCamera;
  private final AprilTagFieldLayout fieldLayout;

  private List<VisionPose> estimatedPoses = new ArrayList<VisionPose>();

  /* Debug Info */
  @AutoLogOutput private int debugTargetsVisible;
  @AutoLogOutput private int debugCurrentTargetId = RobotState.getActiveTargetId();

  @AutoLogOutput
  private String debugCurrentTargetName = RobotState.getTargetName(debugCurrentTargetId);

  @AutoLogOutput private double debugTargetDistance = 0;
  @AutoLogOutput private double debugTargetYaw = 0;
  @AutoLogOutput private Pose2d debugEstimatedPose;

  /* Simulation Support*/
  private boolean simEnabled = false;
  private VisionSystemSim simVision;
  private Supplier<Pose2d> simPoseSupplier;

  public VisionSubsystem(List<VisionCamera> cameras, AprilTagFieldLayout fieldLayout) {
    for (VisionCamera camera : cameras) {
      this.cameras.add(new VisionCameraImpl(camera, fieldLayout));
      estimatedPoses.add(new VisionPose());
    }
    this.fieldLayout = fieldLayout;
    primaryCamera = this.cameras.get(0);
  }

  @Override
  public void enableSimulation(Supplier<Pose2d> poseSupplier, boolean enableWireFrame) {
    simVision = new VisionSystemSim("main");
    simVision.addAprilTags(this.fieldLayout);

    for (VisionCameraImpl camera : cameras) {
      camera.simCamera = new PhotonCameraSim(camera.camera);
      simVision.addCamera(camera.simCamera, camera.robotToCamera);
      camera.simCamera.enableDrawWireframe(enableWireFrame);
    }

    this.simPoseSupplier = poseSupplier;
    simEnabled = true;
  }

  @Override
  public void periodic() {
    if (simEnabled) {
      simVision.update(simPoseSupplier.get());
    }

    for (VisionCameraImpl camera : cameras) {
      camera.update();
      if (camera == primaryCamera) {
        debugTargetsVisible = camera.result.getTargets().size();
      }

      Optional<EstimatedRobotPose> currentEstimatedRobotPose = camera.getEstimatedRobotPose();
      if (currentEstimatedRobotPose.isPresent()) {
        estimatedPoses.get(camera.getIndex()).robotPose =
            currentEstimatedRobotPose.get().estimatedPose.toPose2d();
        estimatedPoses.get(camera.getIndex()).timestamp =
            currentEstimatedRobotPose.get().timestampSeconds;
        estimatedPoses.get(camera.getIndex()).cameraName = camera.getName();

        if (camera == primaryCamera) {
          debugEstimatedPose = estimatedPoses.get(camera.getIndex()).robotPose;
        }
      }
    }

    Optional<Double> distance;
    Optional<Double> yaw;
    if (debugCurrentTargetId != RobotState.getActiveTargetId()) {
      debugCurrentTargetId = RobotState.getActiveTargetId();
      debugTargetDistance = 0;
      debugTargetYaw = 0;
      debugCurrentTargetName = RobotState.getTargetName(debugCurrentTargetId);
    }
    distance = getDistanceToAprilTag(debugCurrentTargetId);
    yaw = getYawToAprilTag(debugCurrentTargetId);
    if (distance.isPresent()) {
      debugTargetDistance = distance.get();
    }
    if (yaw.isPresent()) {
      debugTargetYaw = yaw.get();
    }
  }

  private PhotonTrackedTarget findAprilTag(int id) {
    for (PhotonTrackedTarget target : primaryCamera.getTargets()) {
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
              primaryCamera.getRobotToCamera().getZ(),
              fieldLayout.getTagPose(target.getFiducialId()).get().getZ(),
              -primaryCamera.getRobotToCamera().getRotation().getY(),
              Units.degreesToRadians(target.getPitch())));
    }
    return Optional.empty();
  }

  @Override
  public Optional<Double> getYawToBestTarget() {
    if (primaryCamera.hasTargets()) {
      return Optional.of(primaryCamera.getBestTarget().getYaw());
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

  @Override
  public List<VisionPose> getEstimatedRobotPoses() {
    return estimatedPoses;
  }

  @Override
  public boolean setPrimaryCamera(String name) {
    boolean foundCamera = false;

    for (VisionCameraImpl camera : cameras) {
      if (camera.getName().equals(name)) {
        primaryCamera = camera;
        foundCamera = true;
        break;
      }
    }
    return foundCamera;
  }
}
