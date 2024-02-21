package frc.robot.config;

import com.pathplanner.lib.auto.AutoBuilder;
import frc.robot.subsystems.arm.ArmIOSparkMax;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.climber.ClimberIOSparkMax;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.drive.DriveSwerveYAGSL;
import frc.robot.subsystems.intake.IntakeBase;
import frc.robot.subsystems.intake.IntakeIOTalonSRX;
import frc.robot.subsystems.shooter.ShooterIOSparkMax;
import frc.robot.subsystems.shooter.ShooterSubsystem;

/* Override Inferno specific constants here */
public class RobotConfigInferno extends RobotConfig {
  public RobotConfigInferno() {
    super(false, false, false, false, false, false);

    // Inferno has a Swerve drive train
    // TODO: set DriveConstants.maxVelocityMetersPerSec
    drive = new DriveSwerveYAGSL("yagsl/inferno");
    autoChooser = AutoBuilder.buildAutoChooser("Mobility Auto");

    // Sherman has a TalonSRX based intake
    IntakeConstants.intakeSpeedInVolts = 6.0;
    IntakeConstants.indexSpeedInVolts = 6.0;
    IntakeConstants.feedSpeedInVolts = 6.0;

    intake = new IntakeBase(new IntakeIOTalonSRX(3));

    // Sherman has a dual SparkMax based shooter

    // Values from Carter's Shooter SysId Run on Sherman 2024-02-07
    ShooterConstants.ffKs = 0.08134;
    ShooterConstants.ffKv = 0.019999;
    ShooterConstants.ffKa = 0.0054252;

    ShooterConstants.pidKp = 0.0010514;
    ShooterConstants.pidKi = 0.0;
    ShooterConstants.pidKd = 0.0;
    ShooterConstants.pidVelocityErrorInRPMS = 20;

    ShooterConstants.velocityInRPMs = 3000;
    shooter = new ShooterSubsystem(new ShooterIOSparkMax(2));

    ArmConstants.absolutePositionOffset = 0.42925653052604144; // This is place holder

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

    arm = new ArmSubsystem(new ArmIOSparkMax(4));

    climber = new ClimberSubsystem(new ClimberIOSparkMax(6), new ClimberIOSparkMax(7));
    ClimberConstants.maxPositionInRadians = 4;
    // 16 inchea extension, 5/8 radius from the center of the spoke
    // formula is 16/ 2 * pi * 5/8 to get 4.07 that rounds 4 (SIG FIGS )
  }
}
