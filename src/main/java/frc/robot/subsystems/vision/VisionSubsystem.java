package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.VisionIO.VisionIOInputs;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionSubsystem extends SubsystemBase implements Vision {
  private final VisionIO io;
  //  private final VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();
  private final VisionIOInputs inputs = new VisionIOInputs();
  AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

  public VisionSubsystem(VisionIO io) {
    this.io = io;
  }

  private PhotonTrackedTarget findAprilTag(int aprilTagId) {
    for (int i = 0; i < inputs.targets.length; i++) {
      if (((PhotonTrackedTarget) inputs.targets[i]).getFiducialId() == aprilTagId) {
        return (PhotonTrackedTarget) inputs.targets[i];
      }
    }

    return null;
  }

  @Override
  public double getDistanceToAprilTag(int aprilTagId) {
    Transform3d robotToCameraPose = io.getRobotToCameraPose();
    PhotonTrackedTarget target = findAprilTag(aprilTagId);

    if ((robotToCameraPose != null) && (target != null)) {
      return PhotonUtils.calculateDistanceToTargetMeters(
          robotToCameraPose.getZ(),
          aprilTagFieldLayout.getTagPose(target.getFiducialId()).get().getZ(),
          -robotToCameraPose.getRotation().getY(),
          Units.degreesToRadians(target.getPitch()));
    }

    return -1;
  }

  /*
  public Pose3d getPose3dToAprilTag(int aprilTagId) {
      PhotonTrackedTarget target = findAprilTag(aprilTagId);
      if (target != null)
      {
          return PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(), aprilTagFieldLayout.getTagPose(target.getFiducialId()).get(), io.getRobotToCameraPose());        }
      return null;
  }
  */

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    //    Logger.processInputs("Vision", inputs);
  }
}
