package frc.robot.config;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.assist.PrepareForIntake;
import frc.robot.commands.auto.AutoPrepareForScore;
import frc.robot.commands.auto.AutoScore;
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

  private double translateForAlliance(double angle) {
    var alliance = DriverStation.getAlliance();

    if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
      angle += 90;
    }
    return angle;
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
            new AutoScore(
                RobotConfig.drive,
                RobotConfig.arm,
                RobotConfig.intake,
                RobotConfig.shooter,
                () -> RobotConfig.drive.getAngle(),
                () -> AutoConstants.scoreFromSpeakerAmpSide.armAngleInDegrees,
                () -> AutoConstants.scoreFromSpeakerAmpSide.shooterVelocityInRPMs),
            new PrintCommand("  END: Shoot Piece from Speaker Amp Side")));

    NamedCommands.registerCommand(
        "Shoot Piece from Speaker Center Side",
        new SequentialCommandGroup(
            new PrintCommand("START: Shoot Piece from Speaker Center Side"),
            new AutoScore(
                RobotConfig.drive,
                RobotConfig.arm,
                RobotConfig.intake,
                RobotConfig.shooter,
                () -> RobotConfig.drive.getAngle(),
                () -> AutoConstants.scoreFromSpeakerCenterSide.armAngleInDegrees,
                () -> AutoConstants.scoreFromSpeakerCenterSide.shooterVelocityInRPMs),
            new PrintCommand("  END: Shoot Piece from Speaker Center Side")));        
    
    NamedCommands.registerCommand(
        "Shoot Piece from Speaker Source Side",
        new SequentialCommandGroup(
            new PrintCommand("START: Shoot Piece from Speaker Source Side"),
            new AutoScore(
                RobotConfig.drive,
                RobotConfig.arm,
                RobotConfig.intake,
                RobotConfig.shooter,
                () -> RobotConfig.drive.getAngle(),
                () -> AutoConstants.scoreFromSpeakerSourceSide.armAngleInDegrees,
                () -> AutoConstants.scoreFromSpeakerSourceSide.shooterVelocityInRPMs),
            new PrintCommand("  END: Shoot Piece from Speaker Source Side")));        

    NamedCommands.registerCommand(
        "Shoot Piece from Note Amp Side",
        new SequentialCommandGroup(
            new PrintCommand("START: Shoot Piece from Speaker Amp Side"),
            new AutoScore(
                RobotConfig.drive,
                RobotConfig.arm,
                RobotConfig.intake,
                RobotConfig.shooter,
                () -> translateForAlliance(AutoConstants.scoreFromNoteAmpSide.robotYawInDegrees),
                () -> AutoConstants.scoreFromNoteAmpSide.armAngleInDegrees,
                () -> AutoConstants.scoreFromNoteAmpSide.shooterVelocityInRPMs),
            new PrintCommand("  END: Shoot Piece from Speaker Amp Side")));

    NamedCommands.registerCommand(
        "Shoot Piece from Note Center Side",
        new SequentialCommandGroup(
            new PrintCommand("START: Shoot Piece from Speaker Center Side"),
            new AutoScore(
                RobotConfig.drive,
                RobotConfig.arm,
                RobotConfig.intake,
                RobotConfig.shooter,
                () -> translateForAlliance(AutoConstants.scoreFromNoteCenterSide.robotYawInDegrees),
                () -> AutoConstants.scoreFromNoteCenterSide.armAngleInDegrees,
                () -> AutoConstants.scoreFromNoteCenterSide.shooterVelocityInRPMs),
            new PrintCommand("  END: Shoot Piece from Speaker Center Side")));
            
    NamedCommands.registerCommand(
        "Shoot Piece from Note Source Side",
        new SequentialCommandGroup(
            new PrintCommand("START: Shoot Piece from Speaker Source Side"),
            new AutoScore(
                RobotConfig.drive,
                RobotConfig.arm,
                RobotConfig.intake,
                RobotConfig.shooter,
                () -> translateForAlliance(AutoConstants.scoreFromNoteSourceSide.robotYawInDegrees),
                () -> AutoConstants.scoreFromNoteSourceSide.armAngleInDegrees,
                () -> AutoConstants.scoreFromNoteSourceSide.shooterVelocityInRPMs),
            new PrintCommand("  END: Shoot Piece from Speaker Source Side")));

    NamedCommands.registerCommand(
        "Prepare to Score from Amp Note",
        new SequentialCommandGroup(
            new PrintCommand("START: Prepare to Score from Amp Note"),
            new AutoPrepareForScore(
                RobotConfig.arm,
                RobotConfig.shooter,
                () -> AutoConstants.scoreFromNoteAmpSide.armAngleInDegrees,
                () -> AutoConstants.scoreFromNoteAmpSide.shooterVelocityInRPMs),
            new PrintCommand("  END: Prepare to Score from Amp Note")));
    
    NamedCommands.registerCommand(
        "Prepare to Score from Source Note",
        new SequentialCommandGroup(
            new PrintCommand("START: Prepare to Score from Source Note"),
            new AutoPrepareForScore(
                RobotConfig.arm,
                RobotConfig.shooter,
                () -> AutoConstants.scoreFromNoteSourceSide.armAngleInDegrees,
                () -> AutoConstants.scoreFromNoteSourceSide.shooterVelocityInRPMs),
            new PrintCommand("  END: Prepare to Score from Source Note")));
  }
}
