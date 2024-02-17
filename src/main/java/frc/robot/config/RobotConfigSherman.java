package frc.robot.config;

import frc.robot.subsystems.arm.ArmIOSparkMax;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.drive.DriveTrain;
import frc.robot.subsystems.intake.IntakeBase;
import frc.robot.subsystems.intake.IntakeIOTalonSRX;
import frc.robot.subsystems.shooter.ShooterIOSparkMax;
import frc.robot.subsystems.shooter.ShooterSubsystem;

/* Override Sherman specific constants here */
public class RobotConfigSherman extends RobotConfig {
  public RobotConfigSherman() {
    super(false, false, false, false, true);

    // Sherman has a tank drive train
    // TODO: set DriveConstants.maxVelocityMetersPerSec
    drive = new DriveTrain();

    // Sherman has a TalonSRX based intake
    IntakeConstants.intakeSpeedInVolts = 6.0;
    IntakeConstants.indexSpeedInVolts = 6.0;
    IntakeConstants.feedSpeedInVolts = 6.0;

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

    ShooterConstants.velocityInRPMs = 3000;
    shooter = new ShooterSubsystem(new ShooterIOSparkMax(2), new ShooterIOSparkMax(1));

    ArmConstants.absolutePositionOffset = 0.4154156603853915; // This is place holder

    ArmConstants.pidKp = 0.1;
    ArmConstants.pidKi = 0.0;
    ArmConstants.pidKd = 0.0;
    ArmConstants.ffKs = 0.0;
    ArmConstants.ffKg = 0.72;
    ArmConstants.ffKv = 6.18;
    ArmConstants.ffKa = 0.04;

    ArmConstants.pidAngleErrorInDegrees = 6.0;
    ArmConstants.maxAngleInDegrees = 84.0;
    ArmConstants.minAngleInDegrees = 0.0;
    ArmConstants.intakeAngleInDegrees = 1;
    ArmConstants.shooterAngleInDegrees = 30;

    arm = new ArmSubsystem(new ArmIOSparkMax());
  }
}
