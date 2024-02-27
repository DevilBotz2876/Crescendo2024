package frc.robot.config;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.assist.PrepareForIntake;
import frc.robot.commands.auto.AutoShootPiece;
import frc.robot.subsystems.drive.DriveSwerveYAGSL;

/* Override Phoenix specific constants here */
public class RobotConfigPhoenix extends RobotConfig {
  public RobotConfigPhoenix() {
    super(false, true, true, true, false);

    // Phoenix has a Swerve drive train
    // TODO: set DriveConstants.maxVelocityMetersPerSec
    DriveConstants.baseRadius = 0.350266;
    drive = new DriveSwerveYAGSL("yagsl/phoenix");

    configureNamedCommands();
    autoChooser = AutoBuilder.buildAutoChooser("Sit Still");
  }

  private void configureNamedCommands() {

    NamedCommands.registerCommand(
        "Intake Piece",
        new SequentialCommandGroup(
            new PrintCommand("START: Intake Piece"),
            new PrepareForIntake(RobotConfig.arm, RobotConfig.intake),
            new PrintCommand("  END: Intake Piece")));

    NamedCommands.registerCommand(
        "Shoot Piece from Speaker Amp Side",
        new SequentialCommandGroup(
            new PrintCommand("START: Shoot Piece from Speaker Amp Side"),
            new AutoShootPiece(
                RobotConfig.drive,
                RobotConfig.arm,
                RobotConfig.shooter,
                RobotConfig.intake,
                () -> RobotConfig.drive.getAngle(),
                () -> AutoConstants.scoreFromSpeakerAmpSide.armAngleInDegrees,
                () -> AutoConstants.scoreFromSpeakerAmpSide.shooterVelocityInRPMs),
            new PrintCommand("  END: Shoot Piece from Speaker Amp Side")));
    NamedCommands.registerCommand(
        "Shoot Piece from Note Amp Side",
        new SequentialCommandGroup(
            new PrintCommand("START: Shoot Piece from Speaker Amp Side"),
            new AutoShootPiece(
                RobotConfig.drive,
                RobotConfig.arm,
                RobotConfig.shooter,
                RobotConfig.intake,
                () -> AutoConstants.scoreFromNoteAmpSide.robotYawInDegrees,
                () -> AutoConstants.scoreFromNoteAmpSide.armAngleInDegrees,
                () -> AutoConstants.scoreFromNoteAmpSide.shooterVelocityInRPMs),
            new PrintCommand("  END: Shoot Piece from Speaker Amp Side")));

    NamedCommands.registerCommand(
        "Shoot Piece from Speaker Center",
        new AutoShootPiece(
            RobotConfig.drive,
            RobotConfig.arm,
            RobotConfig.shooter,
            RobotConfig.intake,
            () -> RobotConfig.drive.getAngle(),
            () -> AutoConstants.scoreFromSpeakerCenter.armAngleInDegrees,
            () -> AutoConstants.scoreFromSpeakerCenter.shooterVelocityInRPMs));
    NamedCommands.registerCommand(
        "Shoot Piece from Note Amp Side",
        new AutoShootPiece(
            RobotConfig.drive,
            RobotConfig.arm,
            RobotConfig.shooter,
            RobotConfig.intake,
            () -> AutoConstants.scoreFromNoteCenter.robotYawInDegrees,
            () -> AutoConstants.scoreFromNoteCenter.armAngleInDegrees,
            () -> AutoConstants.scoreFromNoteCenter.shooterVelocityInRPMs));

    NamedCommands.registerCommand(
        "Shoot Piece from Speaker Source Side",
        new AutoShootPiece(
            RobotConfig.drive,
            RobotConfig.arm,
            RobotConfig.shooter,
            RobotConfig.intake,
            () -> RobotConfig.drive.getAngle(),
            () -> AutoConstants.scoreFromSpeakerSourceSide.armAngleInDegrees,
            () -> AutoConstants.scoreFromSpeakerSourceSide.shooterVelocityInRPMs));
    NamedCommands.registerCommand(
        "Shoot Piece from Note Source Side",
        new AutoShootPiece(
            RobotConfig.drive,
            RobotConfig.arm,
            RobotConfig.shooter,
            RobotConfig.intake,
            () -> AutoConstants.scoreFromNoteSourceSide.robotYawInDegrees,
            () -> AutoConstants.scoreFromNoteSourceSide.armAngleInDegrees,
            () -> AutoConstants.scoreFromNoteSourceSide.shooterVelocityInRPMs));
  }
}
