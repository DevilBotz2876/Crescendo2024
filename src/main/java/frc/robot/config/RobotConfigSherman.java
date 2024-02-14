package frc.robot.config;

import frc.robot.subsystems.drive.DriveTrain;
import frc.robot.subsystems.intake.IntakeBase;
import frc.robot.subsystems.intake.IntakeIOTalonSRX;
import frc.robot.subsystems.shooter.ShooterIOSparkMax;
import frc.robot.subsystems.shooter.ShooterSubsystem;

/* Override Sherman specific constants here */
public class RobotConfigSherman extends RobotConfig {
  public RobotConfigSherman() {
    super(false, false, false);

    // Sherman has a tank drive train
    // TODO: set DriveConstants.maxVelocityMetersPerSec
    drive = new DriveTrain();

    // Sherman has a TalonSRX based intake
    intake = new IntakeBase(new IntakeIOTalonSRX());

    // Sherman has a dual SparkMax based shooter

    // Values from Carter's Shooter SysId Run on Sherman 2024-02-07
    ShooterConstants.ffKs = 0.08134;
    ShooterConstants.ffKv = 0.019999;
    ShooterConstants.ffKa = 0.0054252;
    ShooterConstants.ffKsBottom = 0.058262;
    ShooterConstants.ffKvBottom = 0.019495;
    ShooterConstants.ffKaBottom = 0.0048198;

    ShooterConstants.pidKp = 0.0010514;
    ShooterConstants.pidKi = 0.0;
    ShooterConstants.pidKd = 0.0;
    ShooterConstants.pidKpBottom = 0.0001581;
    ShooterConstants.pidKiBottom = 0.0;
    ShooterConstants.pidKdBottom = 0.0;

    shooter = new ShooterSubsystem(new ShooterIOSparkMax(2), new ShooterIOSparkMax(1));
  }
}
