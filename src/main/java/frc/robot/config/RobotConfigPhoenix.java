package frc.robot.config;

import com.pathplanner.lib.auto.AutoBuilder;
import frc.robot.subsystems.drive.DriveSwerveYAGSL;

/* Override Phoenix specific constants here */
public class RobotConfigPhoenix extends RobotConfig {
  public RobotConfigPhoenix() {
    super(false, true, true, true, false);

    // Phoenix has a Swerve drive train
    // TODO: set DriveConstants.maxVelocityMetersPerSec
    drive = new DriveSwerveYAGSL("yagsl/phoenix");
    autoChooser = AutoBuilder.buildAutoChooser("Mobility Auto");
  }
}
