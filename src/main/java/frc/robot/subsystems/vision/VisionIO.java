package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;

public interface VisionIO {
  //  @AutoLog
  public class VisionIOInputs {
    Object[] targets;
    Pose3d estimatedRobotPose;
  }

  /** Updates the set of loggable inputs. */
  public void updateInputs(VisionIOInputs inputs);

  public default Transform3d getRobotToCameraPose() {
    return null;
  }
  ;
}
