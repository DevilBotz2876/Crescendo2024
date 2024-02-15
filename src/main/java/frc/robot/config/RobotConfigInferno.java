package frc.robot.config;

import com.pathplanner.lib.auto.AutoBuilder;
import frc.robot.subsystems.drive.DriveSwerveYAGSL;

/* Override Inferno specific constants here */
public class RobotConfigInferno extends RobotConfig {
  public RobotConfigInferno() {
    super(false, true, true, true, false);

    // Inferno has a Swerve drive train
    // TODO: set DriveConstants.maxVelocityMetersPerSec
    drive = new DriveSwerveYAGSL("yagsl/inferno");
    autoChooser = AutoBuilder.buildAutoChooser("Mobility Auto");
  }
}
