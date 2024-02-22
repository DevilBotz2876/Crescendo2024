package frc.robot.config;

import com.pathplanner.lib.auto.AutoBuilder;
import frc.robot.subsystems.arm.ArmIOSparkMax;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.climber.ClimberIOSparkMax;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.drive.DriveSwerveYAGSL;
import frc.robot.subsystems.intake.IntakeIOTalonSRX;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterIOSparkMax;
import frc.robot.subsystems.shooter.ShooterSubsystem;

/* Override Inferno specific constants here */
public class RobotConfigInferno extends RobotConfig {
  public RobotConfigInferno() {
    super(false, false, false, false, false, false);

    // Inferno has a Swerve drive train
    // TODO: set DriveConstants.maxVelocityMetersPerSec
    DriveConstants.baseRadius = 0.4086;
    drive = new DriveSwerveYAGSL("yagsl/inferno");
    autoChooser = AutoBuilder.buildAutoChooser("Sit Still");

    // Inferno has a TalonSRX based intake
    IntakeConstants.defaultSpeedInVolts = 6.0;
    IntakeConstants.indexSpeedInVolts = 6.0;
    IntakeConstants.feedSpeedInVolts = 6.0;

    intake = new IntakeSubsystem(new IntakeIOTalonSRX(3));

    // Inferno has a single SparkMax based shooter

    // Values from Nilesh's Shooter SysId Run on Inferno 2024-02-22
    ShooterConstants.ffKs = 0.0084987;
    ShooterConstants.ffKv = 0.021041;
    ShooterConstants.ffKa = 0.010693;

    ShooterConstants.pidKp = 0.0072355;
    ShooterConstants.pidKi = 0.0;
    ShooterConstants.pidKd = 0.0;
    ShooterConstants.pidVelocityErrorInRPMS = 20;

    ShooterConstants.velocityInRPMs = 3000;
    shooter = new ShooterSubsystem(new ShooterIOSparkMax(2));

    ArmConstants.absolutePositionOffset = 0.35; // Determined empirically on 2024-02-22

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

    arm = new ArmSubsystem(new ArmIOSparkMax(4, true));

    climber = new ClimberSubsystem(new ClimberIOSparkMax(7, false), new ClimberIOSparkMax(6, true));
    ClimberConstants.maxPositionInRadians = .4;
    // 16 inchea extension, 5/8 radius from the center of the spoke
    // formula is 16/ 2 * pi * 5/8 to get 4.07 that rounds 4 (SIG FIGS )
  }
}
