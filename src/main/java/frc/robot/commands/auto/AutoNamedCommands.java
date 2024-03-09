package frc.robot.commands.auto;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.assist.PrepareForIntake;
import frc.robot.config.RobotConfig;
import frc.robot.config.RobotConfig.ShooterConstants;
import frc.robot.util.RobotState;
import java.util.Optional;

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
    public static AutoScoreConstants scoreFromSpeakerAmpSide =
        new AutoScoreConstants(0.0, 8, ShooterConstants.velocityInRPMs);
    public static AutoScoreConstants scoreFromNoteAmpSide =
        new AutoScoreConstants(40, 8, ShooterConstants.velocityInRPMs);
    public static AutoScoreConstants scoreFromSpeakerCenterSide =
        new AutoScoreConstants(0.0, 8, ShooterConstants.velocityInRPMs);
    public static AutoScoreConstants scoreFromNoteCenterSide =
        new AutoScoreConstants(0.0, 18, ShooterConstants.velocityInRPMs);
    public static AutoScoreConstants scoreFromSpeakerSourceSide =
        new AutoScoreConstants(0.0, 45, ShooterConstants.velocityInRPMs);
    public static AutoScoreConstants scoreFromNoteSourceSide =
        new AutoScoreConstants(320.0, 30.0, ShooterConstants.velocityInRPMs);
    public static AutoScoreConstants scoreFromOutsideSourceSide =
        new AutoScoreConstants(300.0, 15, ShooterConstants.velocityInRPMs);
  }

  public static void configure() {

    NamedCommands.registerCommand(
        "Intake Piece", new PrepareForIntake(RobotConfig.arm, RobotConfig.intake));

    NamedCommands.registerCommand(
        "Shoot Piece from Speaker Amp Side",
        new AutoScore(
            RobotConfig.drive,
            RobotConfig.arm,
            RobotConfig.intake,
            RobotConfig.shooter,
            () -> RobotConfig.drive.getAngle(),
            () -> AutoConstants.scoreFromSpeakerAmpSide.armAngleInDegrees,
            () -> AutoConstants.scoreFromSpeakerAmpSide.shooterVelocityInRPMs));

    NamedCommands.registerCommand(
        "Shoot Piece from Speaker Center Side",
        new AutoScore(
            RobotConfig.drive,
            RobotConfig.arm,
            RobotConfig.intake,
            RobotConfig.shooter,
            () -> RobotConfig.drive.getAngle(),
            () -> getAngleToTarget(AutoConstants.scoreFromSpeakerCenterSide.armAngleInDegrees),
            () -> AutoConstants.scoreFromSpeakerCenterSide.shooterVelocityInRPMs));

    NamedCommands.registerCommand(
        "Shoot Piece from Speaker Source Side",
        new AutoScore(
            RobotConfig.drive,
            RobotConfig.arm,
            RobotConfig.intake,
            RobotConfig.shooter,
            () -> RobotConfig.drive.getAngle(),
            () -> getAngleToTarget(AutoConstants.scoreFromSpeakerSourceSide.armAngleInDegrees),
            () -> AutoConstants.scoreFromSpeakerSourceSide.shooterVelocityInRPMs));

    NamedCommands.registerCommand(
        "Shoot Piece from Note Amp Side",
        new AutoScore(
            RobotConfig.drive,
            RobotConfig.arm,
            RobotConfig.intake,
            RobotConfig.shooter,
            () -> getYawToTarget(AutoConstants.scoreFromNoteAmpSide.robotYawInDegrees),
            () -> getAngleToTarget(AutoConstants.scoreFromNoteAmpSide.armAngleInDegrees),
            () -> AutoConstants.scoreFromNoteAmpSide.shooterVelocityInRPMs));

    NamedCommands.registerCommand(
        "Shoot Piece from Note Center Side",
        new SequentialCommandGroup(
            new AutoScore(
                RobotConfig.drive,
                RobotConfig.arm,
                RobotConfig.intake,
                RobotConfig.shooter,
                () -> getYawToTarget(AutoConstants.scoreFromNoteCenterSide.robotYawInDegrees),
                () -> getAngleToTarget(AutoConstants.scoreFromNoteCenterSide.armAngleInDegrees),
                () -> AutoConstants.scoreFromNoteCenterSide.shooterVelocityInRPMs)));

    NamedCommands.registerCommand(
        "Shoot Piece from Note Source Side",
        new AutoScore(
            RobotConfig.drive,
            RobotConfig.arm,
            RobotConfig.intake,
            RobotConfig.shooter,
            () -> getYawToTarget(AutoConstants.scoreFromNoteSourceSide.robotYawInDegrees),
            () -> getAngleToTarget(AutoConstants.scoreFromNoteSourceSide.armAngleInDegrees),
            () -> AutoConstants.scoreFromNoteSourceSide.shooterVelocityInRPMs));

    NamedCommands.registerCommand(
        "Shoot Piece from Outside Source Side",
        new AutoScore(
            RobotConfig.drive,
            RobotConfig.arm,
            RobotConfig.intake,
            RobotConfig.shooter,
            () -> getYawToTarget(AutoConstants.scoreFromOutsideSourceSide.robotYawInDegrees),
            () -> AutoConstants.scoreFromOutsideSourceSide.armAngleInDegrees,
            () -> AutoConstants.scoreFromOutsideSourceSide.shooterVelocityInRPMs));

    NamedCommands.registerCommand(
        "Prepare to Score from Amp Note",
        new AutoPrepareForScore(
            RobotConfig.arm,
            RobotConfig.shooter,
            () -> AutoConstants.scoreFromNoteAmpSide.armAngleInDegrees,
            () -> AutoConstants.scoreFromNoteAmpSide.shooterVelocityInRPMs));

    NamedCommands.registerCommand(
        "Prepare to Score from Source Note",
        new AutoPrepareForScore(
            RobotConfig.arm,
            RobotConfig.shooter,
            () -> AutoConstants.scoreFromNoteSourceSide.armAngleInDegrees,
            () -> AutoConstants.scoreFromNoteSourceSide.shooterVelocityInRPMs));

    NamedCommands.registerCommand(
        "Prepare to Score from Outside Source Side",
        new AutoPrepareForScore(
            RobotConfig.arm,
            RobotConfig.shooter,
            () -> AutoConstants.scoreFromOutsideSourceSide.armAngleInDegrees,
            () -> AutoConstants.scoreFromOutsideSourceSide.shooterVelocityInRPMs));
  }

  private static double getYawToTarget(double defaultYawToTarget) {
    Optional<Double> getYawToAprilTag =
        RobotConfig.vision.getYawToAprilTag(RobotState.getActiveTargetId());

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
        RobotConfig.vision.getDistanceToAprilTag(RobotState.getActiveTargetId());

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
