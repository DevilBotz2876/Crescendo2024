package frc.robot.controls;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.assist.EjectPiece;
import frc.robot.commands.drive.DriveCommand;
import frc.robot.commands.shooter.SetShooterVelocity;
import frc.robot.commands.vision.AlignToTarget;
import frc.robot.config.RobotConfig;
import frc.robot.config.RobotConfig.ArmConstants;
import frc.robot.config.RobotConfig.DriveConstants;
import frc.robot.config.RobotConfig.ShooterConstants;
import frc.robot.subsystems.vision.VisionCamera;
import frc.robot.util.RobotState;
import frc.robot.util.RobotState.DriveMode;
import frc.robot.util.RobotState.SpeakerShootingMode;
import frc.robot.util.RobotState.TargetMode;
import java.util.Map;
import java.util.Optional;

public class DriverControls {
  public static void setupGUI() {
    int colIndex = 0;
    int rowIndex = 0;
    ShuffleboardTab driverTab = Shuffleboard.getTab("Driver");

    /* Autonomous Chooser */
    driverTab
        .add("Autononmous", RobotConfig.autoChooser)
        .withWidget(BuiltInWidgets.kComboBoxChooser)
        .withPosition(colIndex, rowIndex++)
        .withSize(2, 1);

    driverTab
        .addBoolean("Field Oriented", () -> RobotState.isFieldOriented())
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withPosition(colIndex, rowIndex++)
        .withSize(2, 1);
    colIndex += 2;
    rowIndex = 0;

    for (VisionCamera camera : RobotConfig.cameras) {
      try {
        driverTab
            .addCamera(
                camera.getName() + " cam",
                camera.getName(),
                "mjpg:http://10.28.76.11:" + camera.getPort() + "/?action=stream")
            .withProperties(Map.of("showControls", false))
            .withPosition(colIndex, rowIndex)
            .withSize(3, 3);
      } catch (Exception e) {
        // Do Nothing
      }
      rowIndex += 3;
    }
    colIndex += 3;
    rowIndex = 0;

    driverTab
        .addBoolean("Amp Mode", () -> RobotState.isAmpMode())
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withPosition(colIndex, rowIndex++)
        .withSize(2, 1);

    driverTab
        .addDouble("Abs Angle (degrees)", () -> RobotConfig.arm.getAngle())
        .withWidget(BuiltInWidgets.kNumberBar)
        .withProperties(
            Map.of("min", ArmConstants.minAngleInDegrees, "max", ArmConstants.maxAngleInDegrees))
        .withSize(2, 1)
        .withPosition(colIndex, rowIndex++);

    driverTab
        .addDouble("Rel Angle (degrees)", () -> RobotConfig.arm.getRelativeAngle())
        .withWidget(BuiltInWidgets.kNumberBar)
        .withProperties(
            Map.of("min", ArmConstants.minAngleInDegrees, "max", ArmConstants.maxAngleInDegrees))
        .withSize(2, 1)
        .withPosition(colIndex, rowIndex++);

    ShuffleboardLayout speakerModeLayout =
        driverTab
            .getLayout("Speaker Mode", BuiltInLayouts.kList)
            //            .withProperties(Map.of("Label position", "HIDDEN"))
            .withSize(2, 3)
            .withPosition(colIndex, rowIndex++);

    Command speakerModeFromSubwoofer =
        new InstantCommand(
            () -> RobotState.setSpeakerShootingMode(SpeakerShootingMode.SPEAKER_FROM_SUBWOOFER));
    speakerModeFromSubwoofer.setName("From Subwoofer");

    Command speakerModeFromPodium =
        new InstantCommand(
            () -> RobotState.setSpeakerShootingMode(SpeakerShootingMode.SPEAKER_FROM_PODIUM));
    speakerModeFromPodium.setName("From Podium");

    Command speakerModeVisionBased =
        new InstantCommand(
            () -> RobotState.setSpeakerShootingMode(SpeakerShootingMode.SPEAKER_VISION_BASED));
    speakerModeVisionBased.setName("Vision Based");

    speakerModeLayout
        .addString("Speaker Mode", () -> RobotState.getShootingModeName())
        .withWidget(BuiltInWidgets.kTextView)
        .withPosition(0, 0)
        .withSize(2, 1);

    speakerModeLayout.add(speakerModeFromSubwoofer).withPosition(0, 1);
    speakerModeLayout.add(speakerModeFromPodium).withPosition(0, 2);
    speakerModeLayout.add(speakerModeVisionBased).withPosition(0, 3);
  }

  public static void setupControls(CommandXboxController mainController) {
    setupGUI();
    /* Drive Controls */
    /* Controller 0:
     *     Left Stick Up/Down = *Field Oriented* Drive Away/Towards Driver Station
     *     Left Stick Left/Right = *Field Oriented* Drive Left/Right relative to Driver Station
     *     Right Bumper = Toggles Auto Orient/Prepare For Score
     *     Right Stick Left/Right
     *         Auto Orient Off: Rotate CCW/CW
     *         Auto Orient On: Vision based rotation/arm angle
     */
    RobotState.setDriveMode(DriveMode.FIELD);

    RobotConfig.drive.setDefaultCommand(
        new DriveCommand(
            RobotConfig.drive,
            () -> MathUtil.applyDeadband(-mainController.getLeftY(), 0.05),
            () -> MathUtil.applyDeadband(-mainController.getLeftX(), 0.05),
            () -> MathUtil.applyDeadband(-mainController.getRightX(), 0.05)));

    // TODO: Eventually remove!....this is for debug only
    mainController.back().onTrue(new InstantCommand(() -> RobotConfig.drive.resetOdometry()));
    mainController
        .start()
        .onTrue(
            new InstantCommand(
                () ->
                    RobotConfig.drive.setFieldOrientedDrive(
                        !RobotConfig.drive.isFieldOrientedDrive())));

    /* Climber Controls */
    /*     D-Pad Up = Climber Up
     *     D-Pad Down = Climber Down
     */
    mainController
        .pov(0)
        .onTrue(
            new ParallelCommandGroup(
                RobotConfig.intake.getTurnOffCommand(),
                RobotConfig.shooter.getTurnOffCommand(),
                RobotConfig.arm.getStowCommand(),
                RobotConfig.climber.getExtendCommand()));
    mainController.pov(180).onTrue(RobotConfig.climber.getRetractCommand());

    /* Driver Assist Controls */
    /*     Left Trigger = "Lower Arm to Intake Piece"
     *     Right Trigger = "Score Piece"
     *     Left Bumper = "Eject Piece"
     *     Right Bumper = "Prepare for Score" aka Auto Aim/Orient
     */
    mainController
        .leftTrigger()
        .onTrue(
            new InstantCommand(
                () -> {
                  if (false == RobotConfig.intake.isPieceDetected()) {
                    RobotConfig.intake.turnOn(); // turn on intake, if it isn't already
                    RobotConfig.arm.setAngle(ArmConstants.intakeAngleInDegrees);
                  } else {
                    RobotConfig.arm.setAngle(ArmConstants.stowIntakeAngleInDegrees);
                  }
                },
                RobotConfig.arm));

    mainController
        .leftTrigger()
        .onFalse(
            new InstantCommand(
                () -> RobotConfig.arm.setAngle(ArmConstants.stowIntakeAngleInDegrees),
                RobotConfig.arm));

    mainController
        .rightTrigger()
        .onTrue(
            new SequentialCommandGroup(
                new InstantCommand(
                    () -> RobotConfig.drive.lockPose(),
                    RobotConfig.drive), // lock drive wheels/pose
                new SetShooterVelocity(RobotConfig.shooter, () -> RobotState.getShooterVelocity())
                    .withTimeout(
                        ShooterConstants
                            .pidTimeoutInSeconds), // set shooter velocity in case it's not already
                // on
                RobotConfig.intake.getTurnOnCommand()));

    mainController.leftBumper().onTrue(new EjectPiece(RobotConfig.intake, RobotConfig.arm));

    mainController
        .rightBumper()
        .onTrue(
            new ParallelCommandGroup(
                RobotConfig.intake.getTurnOffCommand(),
                new AlignToTarget(
                        RobotConfig.drive, RobotConfig.vision, () -> RobotState.getActiveTargetId())
                    .withTimeout(DriveConstants.pidTimeoutInSeconds),
                new SetShooterVelocity(RobotConfig.shooter, () -> RobotState.getShooterVelocity())
                    .withTimeout(ShooterConstants.pidTimeoutInSeconds), // turn on shooter
                /* TODO: Use ArmToPositionTP instead of setting arm angle directly */
                new InstantCommand(
                    () -> {
                      if (RobotState.isAmpMode()) {
                        RobotConfig.arm.setAngle(ArmConstants.ampScoreAngleInDegrees);
                      } else {
                        Optional<Double> armAngle = RobotState.getArmAngleToTarget();
                        if (armAngle.isPresent()) {
                          RobotConfig.arm.setAngle((armAngle.get()));
                        }
                      }
                    },
                    RobotConfig.arm) // adjust arm angle based on vision's distance from target
                ));

    /* Target Selection Controls */
    /*     A Button = Amp Mode
     *     B Button = Speaker Mode
     */
    mainController
        .a()
        .onTrue(
            new InstantCommand(
                () -> {
                  RobotState.setTargetMode(TargetMode.AMP);
                }));
    mainController
        .b()
        .onTrue(
            new InstantCommand(
                () -> {
                  RobotState.setTargetMode(TargetMode.SPEAKER);
                }));

    EventLoop eventLoop = CommandScheduler.getInstance().getDefaultButtonLoop();
    BooleanEvent havePiece =
        new BooleanEvent(eventLoop, () -> RobotConfig.intake.isPieceDetected());

    Trigger havePieceTriggerRising = havePiece.rising().castTo(Trigger::new);

    // We've picked up a note
    havePieceTriggerRising.onTrue(
        new SequentialCommandGroup(
            RobotConfig.intake.getTurnOffCommand(), // Turn off intake
            new InstantCommand(
                () -> RobotConfig.arm.setAngle(ArmConstants.stowIntakeAngleInDegrees),
                RobotConfig.arm), // Put arm in stow mode
            new InstantCommand(
                () -> RobotConfig.shooter.runVelocity(RobotState.getShooterVelocity()),
                RobotConfig.shooter) // Turn on the shooter
            ));

    Trigger havePieceTriggerFalling = havePiece.falling().castTo(Trigger::new);

    // We no longer have a note
    havePieceTriggerFalling.onFalse(
        new ParallelCommandGroup(
            RobotConfig.intake.getTurnOnCommand(), // Turn on intake
            new InstantCommand(
                () -> RobotConfig.arm.setAngle(ArmConstants.stowIntakeAngleInDegrees),
                RobotConfig.arm), // Put arm in stow mode
            new InstantCommand(
                () -> RobotConfig.shooter.runVelocity(0), RobotConfig.shooter) // Turn off shooter
            ));
  }
}
