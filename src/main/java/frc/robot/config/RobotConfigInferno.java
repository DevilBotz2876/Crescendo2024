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
import frc.robot.subsystems.intake.IntakeIOTalonSRX;
import frc.robot.subsystems.intake.IntakeSubsystem;
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
    // TODO: set DriveConstants.maxVelocityMetersPerSec
    DriveConstants.anglePidKp = 0.02;
    DriveConstants.anglePidKi = 0.0;
    DriveConstants.anglePidKd = 0.0;
    DriveConstants.pidAngleErrorInDegrees = 0.5;
    drive = new DriveSwerveYAGSL("yagsl/inferno");

    // Inferno has a TalonSRX based intake
    IntakeConstants.defaultSpeedInVolts = 6.0;
    IntakeConstants.indexSpeedInVolts = 4.5;
    IntakeConstants.feedSpeedInVolts = 6.0;

    intake = new IntakeSubsystem(new IntakeIOTalonSRX(3, true));

    // Inferno has a single SparkMax based shooter

    // Values from Nilesh's Shooter SysId Run @ WPI on Inferno 2024-03-09
    ShooterConstants.ffKs =
        0.033556; // SysId calculated -0.036669, but likely erroneous.  Will need to re-run sysid
    ShooterConstants.ffKv = 0.019623;
    ShooterConstants.ffKa = 0.0069448;

    ShooterConstants.pidKp = 0.0037689;
    ShooterConstants.pidKi = 0.0;
    ShooterConstants.pidKd = 0.0;
    ShooterConstants.pidVelocityErrorInRPMS = 100;

    ShooterConstants.ampScoreVelocityInRPMs = 2000;
    ShooterConstants.velocityInRPMs = 4500;
    shooter = new ShooterSubsystem(new ShooterIOSparkMax(2));

    ArmConstants.absolutePositionOffset = 0.362; // Determined empirically on 2024-02-22
    ArmConstants.absoluteEncoderInversion = -1;

    ArmConstants.pidKp = 0.1;
    ArmConstants.pidKi = 0.0;
    ArmConstants.pidKd = 0.0;
    ArmConstants.ffKs = 0.0;
    ArmConstants.ffKg = 0.72;
    ArmConstants.ffKv = 6.18;
    ArmConstants.ffKa = 0.04;

    ArmConstants.maxVelocity = 6.0;
    ArmConstants.maxAcceleration = 8.0;

    ArmConstants.pidAngleErrorInDegrees = 6.0;
    ArmConstants.maxAngleInDegrees = 84.0;

    ArmConstants.minAngleInDegrees = -1.0;
    ArmConstants.intakeAngleInDegrees = 1;
    ArmConstants.ampScoreAngleInDegrees = 80;
    ArmConstants.subwooferScoreAngleInDegrees = 9.80;
    ArmConstants.subwooferScoreFromPodiumAngleInDegrees = 26.5;
    ArmConstants.noteScoreAngleInDegrees = 22.56;
    ArmConstants.stowIntakeAngleInDegrees = 14.64;
    ArmConstants.matchStartArmAngle = 90;

    arm = new ArmSubsystem(new ArmIOSparkMax(4, true));

    ClimberConstants.minPositionInRadians = 0.01;
    ClimberConstants.maxPositionInRadians = 0.47;
    ClimberConstants.defaultSpeedInVolts = 12.0;
    ClimberConstants.autoZeroVoltage = 2.0;
    ClimberConstants.autoZeroMaxCurrent = 16;
    ClimberConstants.autoZeroMinVelocity = 1.0;
    ClimberConstants.autoZeroExtendTimeInSeconds = 0.5;
    ClimberConstants.autoZeroMaxRetractTimeInSeconds =
        10.0 + ClimberConstants.autoZeroExtendTimeInSeconds;
    ClimberConstants.autoZeroOffset =
        -0.5; // When auto-zeroing, to reduce stress on the mechanism, this is the amount we want to
    ClimberConstants.matchStartPositionRight = 0.36;
    // retract the climber after auto-zeroing
    climber = new ClimberSubsystem(new ClimberIOSparkMax(7, false), new ClimberIOSparkMax(6, true));

    cameras = new ArrayList<VisionCamera>();
    /* TODO: Measure and set camera name/location */
    cameras.add(
        new VisionCamera(
            "shooter",
            "1188",
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

  @Override
  public Optional<Double> getArmAngleFromDistance(double distanceInMeters) {
    /* TODO: Insert mapping of distance to arm angle for scoring in speaker */
    System.out.println("TODO: Inferno getArmAngleFromDistance(" + distanceInMeters + ")");
    if (distanceInMeters > 2.0) return Optional.empty();
    return Optional.of(30 * distanceInMeters / 3.0);
  }
}
