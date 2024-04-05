package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.config.RobotConfig;
import frc.robot.config.RobotConfig.ArmConstants;
import frc.robot.config.RobotConfig.ShooterConstants;
import java.util.Optional;

public class DevilBotState {
  public static boolean climberNeedsToBeZeroedAtStart =
      false; // Set to true is the climber is holding up the arm

  public enum TargetMode {
    SPEAKER,
    AMP
  }

  private static TargetMode targetMode = TargetMode.SPEAKER;
  private static String[] targetName = {
    "Unknown", // 0
    "Blue Source - Right", // 1
    "Blue Source - Left", // 2
    "Red Speaker - Right", // 3
    "Red Speaker - Center", // 4
    "Red Amp", // 5
    "Blue Amp", // 6
    "Blue Speaker - Center", // 7
    "Blue Speaker - Left", // 8
    "Red Source - Right", // 9
    "Red Source - Left", // 10
    "Red Stage - Source", // 11
    "Red Stage - Amp", // 12
    "Red Stage - Center Line", // 13
    "Blue Stage - Center Line", // 14
    "Blue Stage - Amp", // 15
    "Blue Stage - Source" // 16
  };

  public static void setTargetMode(TargetMode targetMode) {
    DevilBotState.targetMode = targetMode;
  }

  public static TargetMode getTargetMode() {
    return targetMode;
  }

  public static boolean isAmpMode() {
    return (targetMode == TargetMode.AMP);
  }

  public static boolean isRedAlliance() {
    Optional<Alliance> alliance = DriverStation.getAlliance();

    if (alliance.isPresent()) {
      return (DriverStation.Alliance.Red == alliance.get());
    }

    return false; // Default to blue alliance if driver station doesn't indicate alliance
  }

  public static int getActiveTargetId() {
    DriverStation.Alliance currentAlliance = DriverStation.Alliance.Blue;

    if (isRedAlliance()) {
      currentAlliance = DriverStation.Alliance.Red;
    }

    switch (targetMode) {
      case SPEAKER:
        switch (currentAlliance) {
          case Blue:
            return 7;
          case Red:
            return 4;
          default:
            return 0;
        }

      case AMP:
        switch (currentAlliance) {
          case Blue:
            return 6;
          case Red:
            return 5;
          default:
            return 0;
        }
      default:
        return 0;
    }
  }

  public static String getTargetName(int id) {
    if (id > targetName.length + 1) {
      return targetName[0];
    } else {
      return targetName[id];
    }
  }

  public static double getShooterVelocity() {
    if (isAmpMode()) {
      return ShooterConstants.ampScoreVelocityInRPM;
    } else {
      return ShooterConstants.velocityInRPM;
    }
  }

  public enum DriveMode {
    FIELD,
    ROBOT
  }

  private static DriveMode driveMode = DriveMode.FIELD;

  public static void setDriveMode(DriveMode driveMode) {
    DevilBotState.driveMode = driveMode;

    switch (driveMode) {
      case FIELD:
        RobotConfig.drive.setFieldOrientedDrive(true);
        break;
      case ROBOT:
        RobotConfig.drive.setFieldOrientedDrive(false);
        break;
    }
  }

  public static boolean isFieldOriented() {
    return (driveMode == DriveMode.FIELD);
  }

  public enum SpeakerShootingMode {
    SPEAKER_FROM_SUBWOOFER,
    SPEAKER_FROM_PODIUM,
    SPEAKER_VISION_BASED,
  }

  private static SpeakerShootingMode shootingMode = SpeakerShootingMode.SPEAKER_VISION_BASED;

  public static void setShootingMode(SpeakerShootingMode position) {
    DevilBotState.shootingMode = position;
  }

  public static String getShootingModeName() {
    if (isAmpMode()) {
      return "Amp";
    }

    switch (DevilBotState.shootingMode) {
      case SPEAKER_FROM_SUBWOOFER:
        return "Speaker (Sub)";

      case SPEAKER_FROM_PODIUM:
        return "Speaker (Podium)";

      case SPEAKER_VISION_BASED:
        return "Speaker (Auto)";

      default:
        return "Unknown";
    }
  }

  public static Optional<Double> getArmAngleToTarget() {
    {
      if (isAmpMode()) {
        return Optional.of(ArmConstants.ampScoreAngleInDegrees);
      }

      switch (DevilBotState.shootingMode) {
        case SPEAKER_FROM_SUBWOOFER:
          return Optional.of(ArmConstants.subwooferScoreAngleInDegrees);

        case SPEAKER_FROM_PODIUM:
          return Optional.of(ArmConstants.subwooferScoreFromPodiumAngleInDegrees);

        case SPEAKER_VISION_BASED:
          Optional<Double> distanceToTarget =
              RobotConfig.vision.getDistanceToAprilTag(DevilBotState.getActiveTargetId());
          if (distanceToTarget.isPresent()) {
            Optional<Double> armAngle =
                RobotConfig.instance.getArmAngleFromDistance(distanceToTarget.get());

            return armAngle;
          } else {
            distanceToTarget = RobotConfig.drive.getDistanceFromSpeaker();
            if (distanceToTarget.isPresent()) {
              Optional<Double> armAngle =
                  RobotConfig.instance.getArmAngleFromDistance(distanceToTarget.get());

              return armAngle;
            }
          }
          return Optional.empty();

        default:
          System.err.println("Shooting Mode Not Implemented!");
          return Optional.empty();
      }
    }
  }

  public static double getVisionRobotYawToTarget() {
    double yawToTarget = RobotConfig.drive.getAngle();

    if (DevilBotState.isAmpMode() != true) {
      Optional<Double> visionYawToTarget =
          RobotConfig.vision.getYawToAprilTag(DevilBotState.getActiveTargetId());
      if (visionYawToTarget.isPresent()) {
        yawToTarget -= visionYawToTarget.get();
      }
    }
    return yawToTarget;
  }

  public enum PieceDetectionMode {
    ENABLED,
    DISABLED
  }

  private static PieceDetectionMode pieceDetectionMode = PieceDetectionMode.ENABLED;

  public static boolean isPieceDetectionEnabled() {
    return PieceDetectionMode.ENABLED == pieceDetectionMode;
  }

  public static void setPieceDetectionMode(PieceDetectionMode mode) {
    pieceDetectionMode = mode;
  }

  public enum State {
    UNKNOWN,
    DISABLED,
    AUTO,
    TELEOP,
    TEST
  }

  private static State state = State.UNKNOWN;
  private static boolean stateChanged;

  public static void setState(State state) {
    if (state != DevilBotState.state) {
      stateChanged = true;
    }
    DevilBotState.state = state;
  }

  public static State getState() {
    return DevilBotState.state;
  }

  public static boolean stateChanged() {
    if (stateChanged) {
      stateChanged = false;
      return true;
    }
    return false;
  }
}
