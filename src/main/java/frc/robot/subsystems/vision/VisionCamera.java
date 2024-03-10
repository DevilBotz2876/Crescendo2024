package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Transform3d;

public class VisionCamera {
  final String name;
  /*
   * Location of the camera view relative to the robot (in meters)
   *
   * Translation(x,y,z) - location of the camera relative to the center of the robot (in meters)
   *    x +/- is forward/back from center
   *    y +/- is left/right from center.
   *    z is relative to the ground (and should be positive)
   *  E.g. Translation(0.0, -0.5, 0.5) is 1/2 meter right of center and 1/2 meter above the ground
   *
   * Rotation(roll, pitch, yaw) - orientation of the camera view (in radians)
   *    roll +/- is CCW/CW around x-axis (should generally be 0)
   *    pitch +/- is down/up around y-axis (should generally be <= 0)
   *    yaw +/- is left/right around the z-axis
   *  E.g. Rotation(0, Units.degreesToRadians(-20), Units.degreesToRadians(-90)) is pitched up by 20 degrees, and facing to the right
   */
  Transform3d robotToCamera;
  final String port;

  /**
   * @param cameraName
   * @param robotToCamera
   * @param port
   */
  public VisionCamera(String cameraName, String port, Transform3d robotToCamera) {
    this.name = cameraName;
    this.port = port;
    this.robotToCamera = robotToCamera;
  }

  public VisionCamera(String cameraName, Transform3d robotToCamera) {
    this(cameraName, null, robotToCamera);
  }

  public String getName() {
    return name;
  }

  public String getPort() {
    return port;
  }
}
