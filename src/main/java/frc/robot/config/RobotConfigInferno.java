package frc.robot.config;

import com.pathplanner.lib.auto.AutoBuilder;

import frc.robot.subsystems.climber.ClimberIOSparkMax;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.drive.DriveSwerveYAGSL;


/* Override Inferno specific constants here */
public class RobotConfigInferno extends RobotConfig {
  public RobotConfigInferno() {
    super(false, true, true, true, false);

    // Inferno has a Swerve drive train
    // TODO: set DriveConstants.maxVelocityMetersPerSec
    drive = new DriveSwerveYAGSL("yagsl/inferno");
    autoChooser = AutoBuilder.buildAutoChooser("Mobility Auto");
    climber = new ClimberSubsystem(new ClimberIOSparkMax(6), new ClimberIOSparkMax(7));
    climber.maxPositionInRadians = 4; 
    // 16 inchea extension, 5/8 radius from the center of the spoke
    // formula is 16/ 2 * pi * 5/8 to get 4.07 that rounds 4 (SIG FIGS )
  }
}
