package frc.robot.commands.auto;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
            0.0, ArmConstants.subwooferScoreAngleInDegrees, ShooterConstants.velocityInRPM);
    public static AutoScoreConstants scoreFromCSubwooferCenter =
        new AutoScoreConstants(
            0.0, ArmConstants.subwooferScoreAngleInDegrees, ShooterConstants.velocityInRPM);
    public static AutoScoreConstants scoreFromPSubwooferPodiumSide =
        new AutoScoreConstants(
            0.0, ArmConstants.subwooferScoreAngleInDegrees, ShooterConstants.velocityInRPM);

    public static AutoScoreConstants scoreFrom1WingPodiumNote =
        new AutoScoreConstants(
            -28, ArmConstants.noteScoreAngleInDegrees, ShooterConstants.velocityInRPM);
    public static AutoScoreConstants scoreFrom2WingSpeakerNote =
        new AutoScoreConstants(
            0.0, ArmConstants.noteScoreAngleInDegrees, ShooterConstants.velocityInRPM);
    public static AutoScoreConstants scoreFrom3WingAmpNote =
        new AutoScoreConstants(
            28, ArmConstants.noteScoreAngleInDegrees, ShooterConstants.velocityInRPM);
    public static AutoScoreConstants scoreFromBetween2and3 =
        new AutoScoreConstants(
            11,
            ArmConstants.subwooferScoreFromPodiumAngleInDegrees,
            ShooterConstants.velocityInRPM);
  }

  public static void configure() {

    NamedCommands.registerCommand(
        "Intake Piece", new AutoPrepareForIntake(RobotConfig.arm, RobotConfig.intake));

    NamedCommands.registerCommand(
        "Intake Piece v2.0", new AutoPrepareForIntakeV2(RobotConfig.arm, RobotConfig.intake));

    NamedCommands.registerCommand(
        "Turn off Shooter and Intake",
        new ParallelCommandGroup(
            RobotConfig.shooter.getTurnOffCommand(), RobotConfig.intake.getTurnOffCommand()));

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
            null));
    commandList.add(
        new ScorePieceCommand(
            "C (Subwoofer Center)",
            () -> AutoConstants.scoreFromCSubwooferCenter.armAngleInDegrees,
            () -> AutoConstants.scoreFromCSubwooferCenter.shooterVelocityInRPMs,
            null));
    commandList.add(
        new ScorePieceCommand(
            "P (Subwoofer Podium-Side)",
            () -> AutoConstants.scoreFromPSubwooferPodiumSide.armAngleInDegrees,
            () -> AutoConstants.scoreFromPSubwooferPodiumSide.shooterVelocityInRPMs,
            null));

    /* Vision Assisted yaw/angle*/
    commandList.add(
        new ScorePieceCommand(
            "1 (Wing Podium Note)",
            () -> getArmAngleToTarget(AutoConstants.scoreFrom1WingPodiumNote.armAngleInDegrees),
            () -> AutoConstants.scoreFrom1WingPodiumNote.shooterVelocityInRPMs,
            () -> getRobotYawToTarget(AutoConstants.scoreFrom1WingPodiumNote.robotYawInDegrees)));
    commandList.add(
        new ScorePieceCommand(
            "2 (Wing Speaker Note)",
            () -> getArmAngleToTarget(AutoConstants.scoreFrom2WingSpeakerNote.armAngleInDegrees),
            () -> AutoConstants.scoreFrom2WingSpeakerNote.shooterVelocityInRPMs,
            () -> getRobotYawToTarget(AutoConstants.scoreFrom2WingSpeakerNote.robotYawInDegrees)));
    commandList.add(
        new ScorePieceCommand(
            "3 (Wing Amp Note)",
            () -> getArmAngleToTarget(AutoConstants.scoreFrom3WingAmpNote.armAngleInDegrees),
            () -> AutoConstants.scoreFrom3WingAmpNote.shooterVelocityInRPMs,
            () -> getRobotYawToTarget(AutoConstants.scoreFrom3WingAmpNote.robotYawInDegrees)));
    commandList.add(
        new ScorePieceCommand(
            "Between 2 and 3",
            () -> getArmAngleToTarget(AutoConstants.scoreFromBetween2and3.armAngleInDegrees),
            () -> AutoConstants.scoreFromBetween2and3.shooterVelocityInRPMs,
            () -> getRobotYawToTarget(AutoConstants.scoreFromBetween2and3.robotYawInDegrees)));

    for (ScorePieceCommand command : commandList) {
      String commandName = "Prepare to Score from " + command.location;
      NamedCommands.registerCommand(
          commandName,
          new SequentialCommandGroup(
              new PrintCommand(commandName).onlyIf(() -> Constants.debugCommands),
              new AutoPrepareForScore(
                  RobotConfig.arm,
                  RobotConfig.shooter,
                  command.armAngleInDegrees,
                  command.shooterVelocityInRPMs)));

      commandName = "Score from " + command.location;
      NamedCommands.registerCommand(
          commandName,
          new SequentialCommandGroup(
              new PrintCommand(commandName).onlyIf(() -> Constants.debugCommands),
              new AutoScore(
                  RobotConfig.drive,
                  RobotConfig.arm,
                  RobotConfig.intake,
                  RobotConfig.shooter,
                  command.robotYawInDegrees,
                  command.armAngleInDegrees,
                  command.shooterVelocityInRPMs)));
    }
  }

  private static double getRobotYawToTarget(double defaultYawToTarget) {
    Optional<Double> getYawToAprilTag =
        RobotConfig.vision.getYawToAprilTag(DevilBotState.getActiveTargetId());

    if (getYawToAprilTag.isPresent()) {
      double visionYaw = RobotConfig.drive.getAngle() - getYawToAprilTag.get();
      //      if (Constants.debugCommands)
      {
        System.out.println(
            "Using Vision Yaw "
                + visionYaw
                + " fixedYaw: "
                + translateForAlliance(defaultYawToTarget));
      }
      return visionYaw;
    }

    return translateForAlliance(defaultYawToTarget);
  }

  private static double getArmAngleToTarget(double defaultAngleToTarget) {
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
    if (DevilBotState.isRedAlliance()) {
      angle = 180 - angle;
    }
    return angle;
  }
}
