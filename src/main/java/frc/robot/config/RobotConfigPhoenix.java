package frc.robot.config;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.Robot;
import frc.robot.commands.auto.AutoNamedCommands;
import frc.robot.subsystems.drive.DriveSwerveYAGSL;
import frc.robot.subsystems.vision.VisionCamera;
import frc.robot.subsystems.vision.VisionSubsystem;
import java.util.ArrayList;

/* Override Phoenix specific constants here */
public class RobotConfigPhoenix extends RobotConfig {
  public RobotConfigPhoenix() {
    super(false, true, true, true, false, true, false);

    // Phoenix has a Swerve drive train
    DriveConstants.rotatePidKp = 0.025;
    DriveConstants.rotatePidKi = 0.0;
    DriveConstants.rotatePidKd = 0.0;
    DriveConstants.rotatePidErrorInDegrees = 1;
    drive = new DriveSwerveYAGSL("yagsl/phoenix");

    cameras = new ArrayList<VisionCamera>();
    cameras.add(
        new VisionCamera(
            "shooter",
            "1182",
            new Transform3d(
                new Translation3d(-0.3048, 0, 0.22),
                new Rotation3d(0, Units.degreesToRadians(-30), Units.degreesToRadians(180)))));

    vision = new VisionSubsystem(cameras, AprilTagFields.k2024Crescendo.loadAprilTagLayoutField());

    if (Robot.isSimulation()) {
      vision.enableSimulation(() -> RobotConfig.drive.getPose(), false);
    }

    AutoNamedCommands.configure();
    autoChooser = AutoBuilder.buildAutoChooser("Sit Still");
  }
}
