package frc.robot.commands.auto;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.assist.PrepareForIntake;
import frc.robot.config.RobotConfig;

public class AutoNamedCommands {
  public static class AutoConstants {
    /* TODO: Fill in the values for scoring here */
    public static AutoScoreConstants scoreFromSpeakerAmpSide =
        new AutoScoreConstants(0.0, 45, 3000);
    public static AutoScoreConstants scoreFromNoteAmpSide = new AutoScoreConstants(40, 45, 3000);
    public static AutoScoreConstants scoreFromSpeakerCenterSide =
        new AutoScoreConstants(0.0, 45, 3000);
    public static AutoScoreConstants scoreFromNoteCenterSide =
        new AutoScoreConstants(0.0, 40, 3000);
    public static AutoScoreConstants scoreFromSpeakerSourceSide =
        new AutoScoreConstants(0.0, 45, 3000);
    public static AutoScoreConstants scoreFromNoteSourceSide =
        new AutoScoreConstants(320.0, 30.0, 3000);
    public static AutoScoreConstants scoreFromOutsideSourceSide =
        new AutoScoreConstants(300.0, 15, 3000);
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
            () -> AutoConstants.scoreFromSpeakerCenterSide.armAngleInDegrees,
            () -> AutoConstants.scoreFromSpeakerCenterSide.shooterVelocityInRPMs));

    NamedCommands.registerCommand(
        "Shoot Piece from Speaker Source Side",
        new AutoScore(
            RobotConfig.drive,
            RobotConfig.arm,
            RobotConfig.intake,
            RobotConfig.shooter,
            () -> RobotConfig.drive.getAngle(),
            () -> AutoConstants.scoreFromSpeakerSourceSide.armAngleInDegrees,
            () -> AutoConstants.scoreFromSpeakerSourceSide.shooterVelocityInRPMs));

    NamedCommands.registerCommand(
        "Shoot Piece from Note Amp Side",
        new AutoScore(
            RobotConfig.drive,
            RobotConfig.arm,
            RobotConfig.intake,
            RobotConfig.shooter,
            () -> translateForAlliance(AutoConstants.scoreFromNoteAmpSide.robotYawInDegrees),
            () -> AutoConstants.scoreFromNoteAmpSide.armAngleInDegrees,
            () -> AutoConstants.scoreFromNoteAmpSide.shooterVelocityInRPMs));

    NamedCommands.registerCommand(
        "Shoot Piece from Note Center Side",
        new SequentialCommandGroup(
            new AutoScore(
                RobotConfig.drive,
                RobotConfig.arm,
                RobotConfig.intake,
                RobotConfig.shooter,
                () -> translateForAlliance(AutoConstants.scoreFromNoteCenterSide.robotYawInDegrees),
                () -> AutoConstants.scoreFromNoteCenterSide.armAngleInDegrees,
                () -> AutoConstants.scoreFromNoteCenterSide.shooterVelocityInRPMs)));

    NamedCommands.registerCommand(
        "Shoot Piece from Note Source Side",
        new AutoScore(
            RobotConfig.drive,
            RobotConfig.arm,
            RobotConfig.intake,
            RobotConfig.shooter,
            () -> translateForAlliance(AutoConstants.scoreFromNoteSourceSide.robotYawInDegrees),
            () -> AutoConstants.scoreFromNoteSourceSide.armAngleInDegrees,
            () -> AutoConstants.scoreFromNoteSourceSide.shooterVelocityInRPMs));

    NamedCommands.registerCommand(
        "Shoot Piece from Outside Source Side",
        new AutoScore(
            RobotConfig.drive,
            RobotConfig.arm,
            RobotConfig.intake,
            RobotConfig.shooter,
            () -> translateForAlliance(AutoConstants.scoreFromOutsideSourceSide.robotYawInDegrees),
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

  private static double translateForAlliance(double angle) {
    var alliance = DriverStation.getAlliance();

    if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
      angle += 90;
    }
    return angle;
  }
}
