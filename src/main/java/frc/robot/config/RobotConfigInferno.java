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
import java.util.Optional;

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
    ShooterConstants.pidVelocityErrorInRPMS = 500;

    ShooterConstants.maxVelocityInRPMs = 6000;
    ShooterConstants.maxAccelerationInRPMsSquared = ShooterConstants.maxVelocityInRPMs * 4;
    ShooterConstants.ampScoreVelocityInRPMs = 2500;
    ShooterConstants.velocityInRPMs = 4500;
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

    ArmConstants.maxVelocityInDegreesPerSecond = 90;
    ArmConstants.maxAccelerationInDegreesPerSecondSquared = 240;

    ArmConstants.pidMaxOutput = 6.0;
    ArmConstants.pidMinOutput = -5.0;

    ArmConstants.pidAngleErrorInDegrees = 1.5;
    ArmConstants.maxAngleInDegrees = 89.0;
    ArmConstants.minAngleInDegrees = -1.0;

    ArmConstants.intakeAngleInDegrees = 1.5;
    ArmConstants.ampScoreAngleInDegrees = 89.0;
    ArmConstants.subwooferScoreAngleInDegrees = 9.80;
    ArmConstants.subwooferScoreFromPodiumAngleInDegrees = 33; // min/max = 33.24/34.37
    ArmConstants.noteScoreAngleInDegrees =
        24.24; // Empirically at reading practice field after running 2-note center
    ArmConstants.stowIntakeAngleInDegrees = 14.64;
    ArmConstants.matchStartArmAngle = 90;
    ArmConstants.pidTimeoutInSeconds = 2.0;

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
                new Translation3d(-Units.inchesToMeters(10.5), 0, Units.inchesToMeters(13)),
                new Rotation3d(0, Units.degreesToRadians(-28), Units.degreesToRadians(180)))));

    cameras.add(
        new VisionCamera(
            "intake",
            "1184",
            new Transform3d(
                new Translation3d(0.3048, 0, 0.22),
                new Rotation3d(0, Units.degreesToRadians(-30), Units.degreesToRadians(0)))));

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

  @Override
  public Optional<Double> getArmAngleFromDistance(double distanceInMeters) {
    /* TODO: Insert mapping of distance to arm angle for scoring in speaker */
    System.out.println("TODO: Inferno getArmAngleFromDistance(" + distanceInMeters + ")");
    // return Optional.empty();
    distanceInMeters -= Units.inchesToMeters(38);
    Optional<Double> angle = Optional.empty();
    if (distanceInMeters < 0.6) {
      angle = Optional.of(8.0 + 6.0 / 0.3 * distanceInMeters);
    } else if (distanceInMeters < 1.2) {
      angle = Optional.of(2.0 / 0.3 * distanceInMeters + 16.0);
    } else if (distanceInMeters < 1.5) {
      angle = Optional.of(25.0);
    } else {
      angle = Optional.of(1.0 / 0.3 * distanceInMeters + 21.0);
    }

    if (angle.isPresent() && angle.get() < ArmConstants.subwooferScoreAngleInDegrees) {
      angle = Optional.of(ArmConstants.subwooferScoreAngleInDegrees);
    }

    return angle;
    /*
      if (distanceInMeters > 2.0) return Optional.empty();
      return Optional.of(30 * distanceInMeters / 3.0);
    */
  }
}
