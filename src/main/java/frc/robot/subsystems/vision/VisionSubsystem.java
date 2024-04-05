package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.RobotConfig;
import frc.robot.config.RobotConfig.VisionConstants;
import frc.robot.util.DevilBotState;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
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
    private final String name;

    private PhotonCameraSim simCamera;
    private PhotonPipelineResult result;
    private Optional<EstimatedRobotPose> estimatedRobotPose;

    private VisionCameraImpl(VisionCamera camera, AprilTagFieldLayout fieldLayout) {
      this.name = camera.name;
      this.camera = new PhotonCamera(camera.name);
      this.robotToCamera = camera.robotToCamera;
      this.poseEstimator =
          new PhotonPoseEstimator(
              fieldLayout,
              PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
              this.camera,
              this.robotToCamera);
      this.index = numCameras++;
      update();
    }

    private void update() {
      result = camera.getLatestResult();
      estimatedRobotPose = poseEstimator.update();
      if (estimatedRobotPose.isPresent()) {
        Logger.recordOutput(
            "VisionSubsystem/Camera/" + name + "/Pose2d",
            estimatedRobotPose.get().estimatedPose.toPose2d());
      }
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
  private VisionCameraImpl primaryCamera = null;
  private final AprilTagFieldLayout fieldLayout;

  /* Debug Info */
  @AutoLogOutput private int numTargetsVisible;
  @AutoLogOutput private int selectedTargetId = DevilBotState.getActiveTargetId();
  @AutoLogOutput private String selectedTargetName = DevilBotState.getTargetName(selectedTargetId);
  @AutoLogOutput private double selectedTargetDistance = 0;
  @AutoLogOutput private double selectedTargetYaw = 0;

  /* Simulation Support*/
  private boolean simEnabled = false;
  private VisionSystemSim simVision;
  private Supplier<Pose2d> simPoseSupplier;

  public VisionSubsystem(List<VisionCamera> cameras, AprilTagFieldLayout fieldLayout) {
    for (VisionCamera camera : cameras) {
      this.cameras.add(new VisionCameraImpl(camera, fieldLayout));
    }
    this.fieldLayout = fieldLayout;
    if (0 != cameras.size()) {
      primaryCamera = this.cameras.get(0);
    }
  }

  @Override
  public void enableSimulation(Supplier<Pose2d> poseSupplier, boolean enableWireFrame) {
    simVision = new VisionSystemSim("main");
    simVision.addAprilTags(this.fieldLayout);

    for (VisionCameraImpl camera : cameras) {
      var cameraProp = new SimCameraProperties();
      cameraProp.setCalibration(800, 600, Rotation2d.fromDegrees(70));
      cameraProp.setCalibError(0.35, 0.10);
      cameraProp.setFPS(120);
      cameraProp.setAvgLatencyMs(50);
      cameraProp.setLatencyStdDevMs(15);
      camera.simCamera = new PhotonCameraSim(camera.camera, cameraProp);
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
        numTargetsVisible = camera.result.getTargets().size();

        // if (numTargetsVisible > 0) {
        //   PhotonTrackedTarget target = camera.result.getBestTarget();
        //   Optional<Pose3d> fieldToTarget = fieldLayout.getTagPose(target.getFiducialId());
        //   if (fieldToTarget.isPresent()) {
        //     debugEstimatedPose2d =
        //         PhotonUtils.estimateFieldToRobotAprilTag(
        //                 target.getBestCameraToTarget(), fieldToTarget.get(),
        // camera.robotToCamera)
        //             .toPose2d();
        //   }
        // }
      }

      Optional<EstimatedRobotPose> currentEstimatedRobotPose = camera.getEstimatedRobotPose();
      if (currentEstimatedRobotPose.isPresent()) {
        Optional<Double> distanceToBestTarget =
            getDistanceToAprilTag(camera.getBestTarget().getFiducialId());
        if (distanceToBestTarget.isPresent()) {
          double distance = distanceToBestTarget.get();

          // Add vision measurement to the drivetrain.
          // TODO: clean up this abstraction
          RobotConfig.drive.addVisionMeasurement(
              currentEstimatedRobotPose.get().estimatedPose.toPose2d(),
              currentEstimatedRobotPose.get().timestampSeconds,
              VecBuilder.fill(distance / 2, distance / 2, distance / 2));
        }
      }
    }

    Optional<Double> distance;
    Optional<Double> yaw;
    if (selectedTargetId != DevilBotState.getActiveTargetId()) {
      selectedTargetId = DevilBotState.getActiveTargetId();
      selectedTargetDistance = 0;
      selectedTargetYaw = 0;
      selectedTargetName = DevilBotState.getTargetName(selectedTargetId);
    }
    distance = getDistanceToAprilTag(selectedTargetId);
    yaw = getYawToAprilTag(selectedTargetId);
    if (distance.isPresent()) {
      selectedTargetDistance = distance.get();
    }
    if (yaw.isPresent()) {
      selectedTargetYaw = yaw.get();
    }
  }

  private PhotonTrackedTarget findAprilTag(int id) {
    if (primaryCamera != null) {
      for (PhotonTrackedTarget target : primaryCamera.getTargets()) {
        if (target.getFiducialId() == id) {
          return target;
        }
      }
    }
    return null;
  }

  @Override
  public Optional<Double> getDistanceToAprilTag(int id) {
    PhotonTrackedTarget target = findAprilTag(id);

    if ((target != null) && (primaryCamera != null)) {
      return Optional.of(
          PhotonUtils.calculateDistanceToTargetMeters(
                  primaryCamera.getRobotToCamera().getZ(),
                  fieldLayout.getTagPose(target.getFiducialId()).get().getZ(),
                  -primaryCamera.getRobotToCamera().getRotation().getY(),
                  Units.degreesToRadians(target.getPitch()))
              + VisionConstants.visionDistanceOffsetInMeters);
    }
    return Optional.empty();
  }

  @Override
  public Optional<Integer> getBestTargetId() {
    if ((primaryCamera != null) && (primaryCamera.hasTargets())) {
      return Optional.of(primaryCamera.getBestTarget().getFiducialId());
    }
    return Optional.empty();
  }

  @Override
  public Optional<Double> getYawToBestTarget() {
    if ((primaryCamera != null) && (primaryCamera.hasTargets())) {
      return Optional.of(primaryCamera.getBestTarget().getYaw());
    }
    return Optional.empty();
  }

  @Override
  public Optional<Double> getDistanceToBestTarget() {
    if ((primaryCamera != null) && (primaryCamera.hasTargets())) {
      return getDistanceToAprilTag(primaryCamera.getBestTarget().getFiducialId());
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
