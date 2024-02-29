package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.Optional;

public class RobotState {
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
    RobotState.targetMode = targetMode;
  }

  public static TargetMode getTargetMode() {
    return RobotState.targetMode;
  }

  public static boolean isAmpMode() {
    return (RobotState.targetMode == TargetMode.AMP);
  }

  public static int getActiveTargetId() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    DriverStation.Alliance currentAlliance;

    if (alliance.isPresent()) {
      currentAlliance = alliance.get();
    } else {
      /* Default to blue, but this should never really happen */
      currentAlliance = DriverStation.Alliance.Blue;
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
}
