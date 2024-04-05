package frc.robot.config;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.subsystems.arm.ArmIOStub;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.climber.ClimberIOStub;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.drive.DriveBase;
import frc.robot.subsystems.intake.IntakeIOStub;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.led.LedIOStub;
import frc.robot.subsystems.led.LedSystem;
import frc.robot.subsystems.shooter.ShooterIOStub;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.vision.VisionCamera;
import frc.robot.subsystems.vision.VisionSubsystem;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

/* Put all constants here with reasonable defaults */
public class RobotConfig {
  public static DriveBase drive;
  public static IntakeSubsystem intake;
  public static ShooterSubsystem shooter;
  public static ArmSubsystem arm;
  public static SendableChooser<Command> autoChooser;
  public static ClimberSubsystem climber;
  public static VisionSubsystem vision;
  public static RobotConfig instance;
  public static List<VisionCamera> cameras;
  public static LedSystem led;

  public static class DriveConstants {
    public static double maxVelocityMetersPerSec = 4.5;
    public static double maxAngularVelocityRadiansSec = 2 * Math.PI;

    public static double rotatePidKp = 0.05;
    public static double rotatePidKi = 0.0;
    public static double rotatePidKd = 0.0;
    public static double rotatePidErrorInDegrees = 2.0;
    public static double pidTimeoutInSeconds = 0.5;
    public static double pidSettlingTimeInSeconds = 0.1;

    public static double slewRateLimiterX = 3;
    public static double slewRateLimiterY = 3;
    public static double slewRateLimiterAngle = 3;

    public static double blueSpeakerX = 0.14;
    public static double speakerY = 5.53;
    public static double redSpeakerX = 16.54 - 0.14;
  }

  public static class ArmConstants {
    // TODO: add a value to negate or not the abs encoder reading. On sherman values are pos, on
    // inferno neg.

    public static double absolutePositionOffset = 0; /* 0-1 */
    public static double absoluteEncoderInversion = 1; /* 1 for none, -1 to invert */

    public static double pidKp = 0.1;
    public static double pidKi = 0.0;
    public static double pidKd = 0.0;
    public static double pidMaxOutput = 0.4;
    public static double pidMinOutput = -0.4;

    public static double ffKs = 0.0;
    public static double ffKv = 0.0;
    public static double ffKa = 0.0;
    public static double ffKg = 0.1;

    public static double pidAngleErrorInDegrees = 2.0;
    public static double pidSettlingTimeInSeconds = 0.1;
    public static double pidTimeoutInSeconds = 3.0;

    public static double maxAngleInDegrees = 90.0;
    public static double minAngleInDegrees = 0.0;
    public static double maxVelocityInDegreesPerSecond = 45;
    public static double maxAccelerationInDegreesPerSecondSquared = 120;

    public static double intakeAngleInDegrees = 1;
    public static double ejectAngleInDegrees = 15;
    public static double ampScoreAngleInDegrees = 80;
    public static double subwooferScoreAngleInDegrees = 10;
    public static double subwooferScoreFromPodiumAngleInDegrees = 20;
    public static double noteScoreAngleInDegrees = 25;
    public static double stowIntakeAngleInDegrees = 15;
    public static double matchStartArmAngle = 90;

    public static double defaultSpeedInVolts = 1.0;

    public static double maxBacklashDegrees = 0.0;

    // Arm Angle Calculations
    // Polynomial: y = a*x^2 + b*x + c
    //   y = angle and x = distance
    public static double minDistanceInMeters = 0; // min distance we can aim at
    public static double maxDistanceInMeters = 3.0; // max distance we can aim at
    public static double Ax2 = 0;
    public static double Bx = (maxAngleInDegrees / 2) / maxDistanceInMeters;
    public static double C = 0;
  }

  public static class ShooterConstants {
    /* Feedforward */
    public static double ffKs = 0.088754;
    public static double ffKv = 0.029757;
    public static double ffKa = 0.01281;
    public static double ffKsBottom = 0.11831;
    public static double ffKvBottom = 0.029802;
    public static double ffKaBottom = 0.019246;

    /* PID */
    public static double pidVelocityErrorInRPM = 300;
    public static double pidSettlingTimeInSeconds = 0.1;
    public static double pidTimeoutInSeconds = 2.0;
    public static double pidKp = 0.043566;
    public static double pidKi = 0.0;
    public static double pidKd = 0.0;
    public static double pidKpBottom = 0.04467;
    public static double pidKiBottom = 0.0;
    public static double pidKdBottom = 0.0;

    public static double velocityInRPM = 3000;
    public static double defaultSpeedInVolts = 6.0;
    public static double ampScoreVelocityInRPM = 1000;
    public static double maxVelocityInRPM = 4000;
    public static double maxAccelerationInRPMSquared = maxVelocityInRPM * 4;
  }

  public static class IntakeConstants {
    public static double defaultSpeedInVolts = 6.0;
    public static double sensorDelayFalseToTrueInSeconds = 0.06;
    public static double sensorDelayTrueToFalseInSeconds = 0.1;
    public static double intakeTimeoutInSeconds = 2.0; // max time to wait for piece to be detected
  }

  public static class ClimberConstants {
    public static double minPositionInRadians = 0.0;
    public static double maxPositionInRadians = 4.0;
    public static double defaultSpeedInVolts = 2.0;
    public static double autoZeroVoltage = 2.0;
    public static double autoZeroMaxCurrent = 16;
    public static double autoZeroMinVelocity = 0.2;
    public static double autoZeroExtendTimeInSeconds = 0.5;
    public static double autoZeroMaxRetractTimeInSeconds = 5.0;
    public static double autoZeroOffset =
        -0.5; // When auto-zeroing, to reduce stress on the mechanism, this is the amount we want to
    public static double matchStartPositionRadiansRight = 2.5;
    // retract the climber after auto-zeroing
    public static double maxExtendTimeInSeconds = 5.0;
    public static double maxRetractTimeInSeconds = 8.0;
  }

  public static class LedConstants {
    public static int Led1PWDPort = 0;
    public static int Led1Length = 34;

    public static int Led2PWDPort = 1;
    public static int Led2Length = 60;
  }

  public static class VisionConstants {
    public static double visionDistanceOffsetInMeters =
        0; // Average difference between vision-calculated distance vs actual
  }

  public Optional<Double> getArmAngleFromDistance(double distanceInMeters) {
    // Calculated using https://stats.blue/Stats_Suite/polynomial_regression_calculator.html
    // Based on empirical measurements done on 2024-04-03
    // (https://docs.google.com/spreadsheets/d/17Rh0MyVeME0KEAvkSqZKafnL0LPy9PoqWqOJ7ho49S8)

    distanceInMeters =
        MathUtil.clamp(
            distanceInMeters, ArmConstants.minDistanceInMeters, ArmConstants.maxDistanceInMeters);

    Optional<Double> angle =
        Optional.of(
            ArmConstants.Ax2 * Math.pow(distanceInMeters, 2)
                + ArmConstants.Bx * distanceInMeters
                + ArmConstants.C);

    // System.out.println("getArmAngleFromDistance(" + distanceInMeters + ") = " + angle.get());

    return angle;
  }

  public RobotConfig() {
    this(true, true, true);
  }

  public RobotConfig(boolean stubDrive, boolean stubShooter, boolean stubIntake) {
    this(stubDrive, stubShooter, stubIntake, true, true);
  }

  public RobotConfig(
      boolean stubDrive,
      boolean stubShooter,
      boolean stubIntake,
      boolean stubArm,
      boolean stubAuto) {
    this(stubDrive, stubShooter, stubIntake, stubArm, stubAuto, true);
  }

  public RobotConfig(
      boolean stubDrive,
      boolean stubShooter,
      boolean stubIntake,
      boolean stubArm,
      boolean stubAuto,
      boolean stubClimber) {
    this(stubDrive, stubShooter, stubIntake, stubArm, stubAuto, stubClimber, true);
  }

  public RobotConfig(
      boolean stubDrive,
      boolean stubShooter,
      boolean stubIntake,
      boolean stubArm,
      boolean stubAuto,
      boolean stubClimber,
      boolean stubVision) {
    this(stubDrive, stubShooter, stubIntake, stubArm, stubAuto, stubClimber, stubVision, true);
  }

  public RobotConfig(
      boolean stubDrive,
      boolean stubShooter,
      boolean stubIntake,
      boolean stubArm,
      boolean stubAuto,
      boolean stubClimber,
      boolean stubVision,
      boolean stubLed) {
    instance = this;

    if (stubDrive) {
      drive = new DriveBase();
    }

    if (stubShooter) {
      shooter =
          new ShooterSubsystem(
              new ShooterIOStub(ShooterIOStub.ShooterId.SHOOTER_TOP),
              new ShooterIOStub(ShooterIOStub.ShooterId.SHOOTER_BOTTOM));
    }

    if (stubIntake) {
      intake = new IntakeSubsystem(new IntakeIOStub());
    }

    if (stubArm) {
      arm = new ArmSubsystem(new ArmIOStub());
    }

    if (stubAuto) {
      autoChooser = new SendableChooser<>();
      autoChooser.setDefaultOption("No Auto Routines Specified", Commands.none());
    }

    if (stubClimber) {
      climber = new ClimberSubsystem(new ClimberIOStub(), new ClimberIOStub());
    }

    if (stubVision) {
      cameras = new ArrayList<VisionCamera>();
      if (Robot.isSimulation()) {
        cameras.add(
            new VisionCamera(
                "photonvision",
                new Transform3d(
                    new Translation3d(-0.221, 0, .164),
                    new Rotation3d(0, Units.degreesToRadians(-20), Units.degreesToRadians(180)))));
        cameras.add(
            new VisionCamera(
                "left",
                new Transform3d(
                    new Translation3d(0, 0.221, .164),
                    new Rotation3d(0, Units.degreesToRadians(-20), Units.degreesToRadians(90)))));

        cameras.add(
            new VisionCamera(
                "right",
                new Transform3d(
                    new Translation3d(0, -0.221, .164),
                    new Rotation3d(0, Units.degreesToRadians(-20), Units.degreesToRadians(-90)))));
      }

      vision =
          new VisionSubsystem(cameras, AprilTagFields.k2024Crescendo.loadAprilTagLayoutField());

      if (Robot.isSimulation()) {
        vision.enableSimulation(() -> RobotConfig.drive.getPose(), false);
      }
    }
    if (stubLed) {
      led = new LedSystem(new LedIOStub());
    }
  }
}
