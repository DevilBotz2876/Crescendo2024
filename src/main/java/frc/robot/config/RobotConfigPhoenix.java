package frc.robot.config;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.Robot;
import frc.robot.subsystems.drive.DriveSwerveYAGSL;
import frc.robot.subsystems.vision.VisionCamera;
import frc.robot.subsystems.vision.VisionSubsystem;
import java.util.ArrayList;

/* Override Phoenix specific constants here */
public class RobotConfigPhoenix extends RobotConfig {
  public RobotConfigPhoenix() {
    super(false, true, true, true, false, true, false);

    // Phoenix has a Swerve drive train
    // TODO: set DriveConstants.maxVelocityMetersPerSec
    drive = new DriveSwerveYAGSL("yagsl/phoenix");
    autoChooser = AutoBuilder.buildAutoChooser("Mobility Auto");

    ArrayList<VisionCamera> cameras = new ArrayList<VisionCamera>();
    /* TODO: Measure and set camera name/location */
    cameras.add(
        new VisionCamera(
            "shooter",
            new Transform3d(
                new Translation3d(0.221, 0, .164),
                new Rotation3d(0, Units.degreesToRadians(-20), 0))));

    vision = new VisionSubsystem(cameras, AprilTagFields.k2024Crescendo.loadAprilTagLayoutField());

    if (Robot.isSimulation()) {
      vision.enableSimulation(() -> RobotConfig.drive.getPose(), true);
    }
  }
}
