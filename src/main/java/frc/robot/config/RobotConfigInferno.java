package frc.robot.config;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.Robot;
import frc.robot.commands.auto.AutoNamedCommands;
import frc.robot.subsystems.arm.ArmIOSparkMax;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.climber.ClimberIOSparkMax;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.drive.DriveSwerveYAGSL;
import frc.robot.subsystems.intake.IntakeIOSparkMax;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.led.LedIOWS121b;
import frc.robot.subsystems.led.LedSystem;
import frc.robot.subsystems.shooter.ShooterIOSparkMax;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.vision.VisionCamera;
import frc.robot.subsystems.vision.VisionSubsystem;
import java.util.ArrayList;

/* Override Inferno specific constants here */
public class RobotConfigInferno extends RobotConfig {
  public RobotConfigInferno() {
    super(false, false, false, false, false, false, false);

    // Inferno has a Swerve drive train
    DriveConstants.rotatePidKp = 0.02;
    DriveConstants.rotatePidKi = 0.0;
    DriveConstants.rotatePidKd = 0.0;
    DriveConstants.rotatePidErrorInDegrees = 0.5;
    DriveConstants.pidTimeoutInSeconds = 0.5;
    DriveConstants.pidSettlingTimeInSeconds = 0.5;

    drive = new DriveSwerveYAGSL("yagsl/inferno");

    // Inferno has a TalonSRX based intake
    // IntakeConstants.defaultSpeedInVolts = 6.0;

    // Reading Intake v2.0
    // IntakeConstants.sensorDelayFalseToTrueInSeconds = 0.06;
    IntakeConstants.sensorDelayTrueToFalseInSeconds = 0.1;

    // WPI Intake v1.0
    IntakeConstants.sensorDelayFalseToTrueInSeconds = 0.0;
    // IntakeConstants.sensorDelayTrueToFalseInSeconds = 0.1;

    // intake = new IntakeSubsystem(new IntakeIOTalonSRX(3, true));

    IntakeConstants.defaultSpeedInVolts = 10.0; // SparkMax/NEO based voltage
    intake = new IntakeSubsystem(new IntakeIOSparkMax(3, false));

    // Inferno has a single SparkMax based shooter

    // Values from Nilesh's Shooter SysId Run @ BHS on Inferno 2024-03-18
    ShooterConstants.ffKs =
        0.0; // SysId calculated -0.016149, but likely erroneous.  Will need to re-run sysid
    ShooterConstants.ffKv = 0.021208;
    ShooterConstants.ffKa = 0.0072313;

    ShooterConstants.pidKp = 0.0047154;
    ShooterConstants.pidKi = 0.0;
    ShooterConstants.pidKd = 0.0;
    ShooterConstants.pidVelocityErrorInRPM = 500;

    ShooterConstants.maxVelocityInRPM = 6000;
    ShooterConstants.maxAccelerationInRPMSquared = ShooterConstants.maxVelocityInRPM * 4;
    ShooterConstants.ampScoreVelocityInRPM = 2500;
    ShooterConstants.velocityInRPM = 4500;
    shooter = new ShooterSubsystem(new ShooterIOSparkMax(2));

    ArmConstants.absolutePositionOffset =
        0.5031059375776484; // Determined empirically on 2024-03-15 (after replacing broken abs)
    // encoder)
    ArmConstants.absoluteEncoderInversion = -1;

    ArmConstants.pidKp = 0.1;
    ArmConstants.pidKi = 0.0;
    ArmConstants.pidKd = 0.0;
    ArmConstants.ffKs = 0.0;
    ArmConstants.ffKg = 0.72;
    ArmConstants.ffKv = 6.18;
    ArmConstants.ffKa = 0.04;

    ArmConstants.maxVelocityInDegreesPerSecond = 120;
    ArmConstants.maxAccelerationInDegreesPerSecondSquared = 120;

    ArmConstants.pidMaxOutput = 6.0;
    ArmConstants.pidMinOutput = -5.0;

    ArmConstants.pidAngleErrorInDegrees = 1.5;
    ArmConstants.maxAngleInDegrees = 89.0;
    ArmConstants.minAngleInDegrees = -1.5;

    ArmConstants.intakeAngleInDegrees = -1.5;
    ArmConstants.ampScoreAngleInDegrees = 89.0;
    ArmConstants.subwooferScoreAngleInDegrees = 9.80;
    ArmConstants.subwooferScoreFromPodiumAngleInDegrees = 33; // min/max = 33.24/34.37
    ArmConstants.noteScoreAngleInDegrees =
        24.24; // Empirically at reading practice field after running 2-note center
    ArmConstants.stowIntakeAngleInDegrees = 5.0;
    ArmConstants.matchStartArmAngle = 90;
    ArmConstants.pidTimeoutInSeconds = 2.0;

    ArmConstants.maxBacklashDegrees = 3.0;

    ArmConstants.minDistanceInMeters = Units.inchesToMeters(38);
    ArmConstants.maxDistanceInMeters = 4.0;
    ArmConstants.Ax2 = -3.2;
    ArmConstants.Bx = 23.7;
    ArmConstants.C = -10.3;
    arm = new ArmSubsystem(new ArmIOSparkMax(4, true));

    ClimberConstants.minPositionInRadians = 0.01;
    ClimberConstants.maxPositionInRadians = 27.875;
    ClimberConstants.defaultSpeedInVolts = 12.0;
    ClimberConstants.autoZeroVoltage = 2.0;
    ClimberConstants.autoZeroMaxCurrent = 16;
    ClimberConstants.autoZeroMinVelocity = 1.0;
    ClimberConstants.autoZeroExtendTimeInSeconds = 0.5;
    ClimberConstants.autoZeroMaxRetractTimeInSeconds =
        10.0 + ClimberConstants.autoZeroExtendTimeInSeconds;
    ClimberConstants.autoZeroOffset =
        -0.5; // When auto-zeroing, to reduce stress on the mechanism, this is the amount we want to
    ClimberConstants.matchStartPositionRadiansRight = 21;
    // retract the climber after auto-zeroing
    climber = new ClimberSubsystem(new ClimberIOSparkMax(7, false), new ClimberIOSparkMax(6, true));

    cameras = new ArrayList<VisionCamera>();
    /* TODO: Measure and set camera name/location */
    cameras.add(
        new VisionCamera(
            "shooter",
            "1188",
            new Transform3d(
                new Translation3d(-Units.inchesToMeters(9.5), 0, Units.inchesToMeters(14)),
                new Rotation3d(0, Units.degreesToRadians(-32), Units.degreesToRadians(180)))));

    cameras.add(
        new VisionCamera(
            "intake",
            "1184",
            new Transform3d(
                new Translation3d(0.3048, 0, 0.22),
                new Rotation3d(0, Units.degreesToRadians(-30), Units.degreesToRadians(0)))));

    cameras.add(
        new VisionCamera(
            "right",
            "1196",
            new Transform3d(
                new Translation3d(
                    -Units.inchesToMeters(5.25),
                    -Units.inchesToMeters(11.25),
                    Units.inchesToMeters(7)),
                new Rotation3d(0, Units.degreesToRadians(-32), Units.degreesToRadians(-90)))));

    cameras.add(
        new VisionCamera(
            "left",
            "1190",
            new Transform3d(
                new Translation3d(
                    -Units.inchesToMeters(5.25),
                    Units.inchesToMeters(11.25),
                    Units.inchesToMeters(7)),
                new Rotation3d(0, Units.degreesToRadians(-32), Units.degreesToRadians(90)))));

    VisionConstants.visionDistanceOffsetInMeters = -0.2;
    vision = new VisionSubsystem(cameras, AprilTagFields.k2024Crescendo.loadAprilTagLayoutField());

    if (Robot.isSimulation()) {
      vision.enableSimulation(() -> RobotConfig.drive.getPose(), false);
    }

    AutoNamedCommands.configure();
    autoChooser = AutoBuilder.buildAutoChooser("Sit Still");

    LedConstants.Led1PWDPort = 9;
    LedConstants.Led1Length = 34;
    led = new LedSystem(new LedIOWS121b());
  }
}
