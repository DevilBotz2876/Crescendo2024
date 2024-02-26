package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Transform3d;

public class VisionCamera {
  final String name;

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
  Transform3d robotToCamera;

  /**
   * @param cameraName
   * @param robotToCamera
   */
  public VisionCamera(String cameraName, Transform3d robotToCamera) {
    this.name = cameraName;
    this.robotToCamera = robotToCamera;
  }
}
