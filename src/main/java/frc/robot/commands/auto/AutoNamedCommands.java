package frc.robot.commands.auto;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.config.RobotConfig;
import frc.robot.config.RobotConfig.ArmConstants;
import frc.robot.config.RobotConfig.ShooterConstants;
import frc.robot.util.DevilBotState;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.DoubleSupplier;

public class AutoNamedCommands {
  public static class AutoScoreConstants {
    public double robotYawInDegrees;
    public double armAngleInDegrees;
    public double shooterVelocityInRPMs;

    public AutoScoreConstants(
        double robotYawInDegrees, double armAngleInDegrees, double shooterVelocityInRPMs) {
      this.robotYawInDegrees = robotYawInDegrees;
      this.armAngleInDegrees = armAngleInDegrees;
      this.shooterVelocityInRPMs = shooterVelocityInRPMs;
    }
  }

  public static class AutoConstants {
    /* TODO: Fill in the values for scoring here */
    public static AutoScoreConstants scoreFromASubwooferAmpSide =
        new AutoScoreConstants(
            0.0, ArmConstants.subwooferScoreAngleInDegrees, ShooterConstants.velocityInRPMs);
    public static AutoScoreConstants scoreFromCSubwooferCenter =
        new AutoScoreConstants(
            0.0, ArmConstants.subwooferScoreAngleInDegrees, ShooterConstants.velocityInRPMs);
    public static AutoScoreConstants scoreFromPSubwooferPodiumSide =
        new AutoScoreConstants(
            0.0, ArmConstants.subwooferScoreAngleInDegrees, ShooterConstants.velocityInRPMs);
    public static AutoScoreConstants scoreFrom3WingAmpNote =
        new AutoScoreConstants(40, 8, ShooterConstants.velocityInRPMs);
    public static AutoScoreConstants scoreFromBetween2and3 =
        new AutoScoreConstants(35, 8, ShooterConstants.velocityInRPMs);
    public static AutoScoreConstants scoreFrom2WingSpeakerNote =
        new AutoScoreConstants(
            0.0, ArmConstants.noteScoreAngleInDegrees, ShooterConstants.velocityInRPMs);
    public static AutoScoreConstants scoreFromNoteSourceSide =
        new AutoScoreConstants(320.0, 30.0, ShooterConstants.velocityInRPMs);
    public static AutoScoreConstants scoreFromOutsideSourceSide =
        new AutoScoreConstants(300.0, 15, ShooterConstants.velocityInRPMs);
    public static AutoScoreConstants scoreFrom1WingPodiumNote =
        new AutoScoreConstants(300, 15, ShooterConstants.velocityInRPMs);
  }

  public static void configure() {

    NamedCommands.registerCommand(
        "Intake Piece", new AutoPrepareForIntake(RobotConfig.arm, RobotConfig.intake));

    /* TODO: merge AutoScoreConstants and ScorePieceCommand */
    class ScorePieceCommand {
      String location;
      DoubleSupplier armAngleInDegrees;
      DoubleSupplier shooterVelocityInRPMs;
      DoubleSupplier robotYawInDegrees;

      ScorePieceCommand(
          String location,
          DoubleSupplier armAngleInDegrees,
          DoubleSupplier shooterVelocityInRPMs,
          DoubleSupplier robotYawInDegrees) {
        this.location = location;
        this.armAngleInDegrees = armAngleInDegrees;
        this.shooterVelocityInRPMs = shooterVelocityInRPMs;
        this.robotYawInDegrees = robotYawInDegrees;
      }
    }

    List<ScorePieceCommand> commandList = new ArrayList<ScorePieceCommand>();
    /* Hard Coded yaw/angle */
    commandList.add(
        new ScorePieceCommand(
            "A (Subwoofer Amp-Side)",
            () -> AutoConstants.scoreFromASubwooferAmpSide.armAngleInDegrees,
            () -> AutoConstants.scoreFromASubwooferAmpSide.shooterVelocityInRPMs,
            () -> RobotConfig.drive.getAngle()));
    commandList.add(
        new ScorePieceCommand(
            "C (Subwoofer Center)",
            () -> AutoConstants.scoreFromCSubwooferCenter.armAngleInDegrees,
            () -> AutoConstants.scoreFromCSubwooferCenter.shooterVelocityInRPMs,
            () -> RobotConfig.drive.getAngle()));
    commandList.add(
        new ScorePieceCommand(
            "P (Subwoofer Podium-Side)",
            () -> AutoConstants.scoreFromPSubwooferPodiumSide.armAngleInDegrees,
            () -> AutoConstants.scoreFromPSubwooferPodiumSide.shooterVelocityInRPMs,
            () -> RobotConfig.drive.getAngle()));

    /* Vision Assisted yaw/angle*/
    commandList.add(
        new ScorePieceCommand(
            "1 (Wing Podium Note)",
            () -> getAngleToTarget(AutoConstants.scoreFrom1WingPodiumNote.armAngleInDegrees),
            () -> AutoConstants.scoreFrom1WingPodiumNote.shooterVelocityInRPMs,
            () -> getYawToTarget(AutoConstants.scoreFrom1WingPodiumNote.robotYawInDegrees)));
    commandList.add(
        new ScorePieceCommand(
            "2 (Wing Speaker Note)",
            () -> getAngleToTarget(AutoConstants.scoreFrom2WingSpeakerNote.armAngleInDegrees),
            () -> AutoConstants.scoreFrom2WingSpeakerNote.shooterVelocityInRPMs,
            () -> getYawToTarget(AutoConstants.scoreFrom2WingSpeakerNote.robotYawInDegrees)));
    commandList.add(
        new ScorePieceCommand(
            "3 (Wing Amp Note)",
            () -> getAngleToTarget(AutoConstants.scoreFrom3WingAmpNote.armAngleInDegrees),
            () -> AutoConstants.scoreFrom3WingAmpNote.shooterVelocityInRPMs,
            () -> getYawToTarget(AutoConstants.scoreFrom3WingAmpNote.robotYawInDegrees)));
    commandList.add(
        new ScorePieceCommand(
            "Between 2 and 3",
            () -> getAngleToTarget(AutoConstants.scoreFromBetween2and3.armAngleInDegrees),
            () -> AutoConstants.scoreFromBetween2and3.shooterVelocityInRPMs,
            () -> getYawToTarget(AutoConstants.scoreFromBetween2and3.robotYawInDegrees)));

    for (ScorePieceCommand command : commandList) {
      NamedCommands.registerCommand(
          "Prepare to Score from " + command.location,
          new AutoPrepareForScore(
              RobotConfig.arm,
              RobotConfig.shooter,
              command.armAngleInDegrees,
              command.shooterVelocityInRPMs));

      NamedCommands.registerCommand(
          "Score from " + command.location,
          new AutoScore(
              RobotConfig.drive,
              RobotConfig.arm,
              RobotConfig.intake,
              RobotConfig.shooter,
              command.robotYawInDegrees,
              command.armAngleInDegrees,
              command.shooterVelocityInRPMs));
    }
  }

  private static double getYawToTarget(double defaultYawToTarget) {
    Optional<Double> getYawToAprilTag =
        RobotConfig.vision.getYawToAprilTag(DevilBotState.getActiveTargetId());

    if (getYawToAprilTag.isPresent()) {
      double visionYaw = RobotConfig.drive.getAngle() - getYawToAprilTag.get();
      if (Constants.debugCommands) {
        System.out.println("Using Vision Yaw " + visionYaw + " fixedYaw: " + defaultYawToTarget);
      }
      return visionYaw;
    }

    return translateForAlliance(defaultYawToTarget);
  }

  private static double getAngleToTarget(double defaultAngleToTarget) {
    Optional<Double> getDistanceToAprilTag =
        RobotConfig.vision.getDistanceToAprilTag(DevilBotState.getActiveTargetId());

    if (getDistanceToAprilTag.isPresent()) {
      double distance = getDistanceToAprilTag.get();
      Optional<Double> armAngle = RobotConfig.instance.getArmAngleFromDistance(distance);
      if (armAngle.isPresent()) {
        double visionAngle = armAngle.get();
        if (Constants.debugCommands) {
          System.out.println(
              "Using Vision Angle "
                  + visionAngle
                  + " distance: "
                  + distance
                  + " fixedAngle: "
                  + defaultAngleToTarget);
        }
        return armAngle.get();
      }
    }
    return defaultAngleToTarget;
  }

  private static double translateForAlliance(double angle) {
    var alliance = DriverStation.getAlliance();

    if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
      angle += 90;
    }
    return angle;
  }
}
