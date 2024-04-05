package frc.robot.controls;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.arm.ArmCommand;
import frc.robot.commands.assist.EjectPiece;
import frc.robot.commands.drive.DriveCommand;
import frc.robot.commands.drive.DriveToYaw;
import frc.robot.commands.shooter.SetShooterVelocity;
import frc.robot.config.RobotConfig;
import frc.robot.config.RobotConfig.ArmConstants;
import frc.robot.config.RobotConfig.DriveConstants;
import frc.robot.config.RobotConfig.IntakeConstants;
import frc.robot.config.RobotConfig.ShooterConstants;
import frc.robot.subsystems.vision.VisionCamera;
import frc.robot.util.DevilBotState;
import frc.robot.util.DevilBotState.DriveMode;
import frc.robot.util.DevilBotState.PieceDetectionMode;
import frc.robot.util.DevilBotState.SpeakerShootingMode;
import frc.robot.util.DevilBotState.State;
import frc.robot.util.DevilBotState.TargetMode;
import java.util.Map;
import java.util.Optional;

public class DriverControls {
  public static CommandXboxController mainController;
  public static CommandXboxController secondaryController;

  static Command ampModeCommand =
      new InstantCommand(() -> DevilBotState.setTargetMode(TargetMode.AMP));
  static Command speakerModeCommand =
      new InstantCommand(() -> DevilBotState.setTargetMode(TargetMode.SPEAKER));

  static Command speakerModeFromSubwoofer =
      new InstantCommand(
          () -> {
            DevilBotState.setTargetMode(TargetMode.SPEAKER);
            DevilBotState.setShootingMode(SpeakerShootingMode.SPEAKER_FROM_SUBWOOFER);
          });

  static Command speakerModeFromPodium =
      new InstantCommand(
          () -> {
            DevilBotState.setTargetMode(TargetMode.SPEAKER);
            DevilBotState.setShootingMode(SpeakerShootingMode.SPEAKER_FROM_PODIUM);
          });

  static Command speakerModeVisionBased =
      new InstantCommand(
          () -> {
            DevilBotState.setTargetMode(TargetMode.SPEAKER);
            DevilBotState.setShootingMode(SpeakerShootingMode.SPEAKER_VISION_BASED);
          });

  public static void setupControls() {
    DriverStation.silenceJoystickConnectionWarning(true);

    mainController = new CommandXboxController(0);
    secondaryController = new CommandXboxController(1);

    setupMainControls(mainController);
    setupCommonControls(mainController);

    if (null != secondaryController) {
      setupSecondaryControls(secondaryController);
      setupCommonControls(secondaryController);
    }

    setupGUI();
    setupLedControls();
  }

  private static void setupGUI() {
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
        .addBoolean("Field Oriented", () -> DevilBotState.isFieldOriented())
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withPosition(colIndex, rowIndex++)
        .withSize(2, 1);

    driverTab
        .addBoolean("Piece Detected", () -> RobotConfig.intake.isPieceDetected())
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withProperties(Map.of("Color when false", "black", "Color when true", "orange"))
        .withPosition(colIndex, rowIndex++)
        .withSize(2, 1);
    colIndex += 2;

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
        .addDouble("Target Angle (degrees)", () -> RobotConfig.arm.getTargetAngle())
        .withWidget(BuiltInWidgets.kNumberBar)
        .withProperties(
            Map.of("min", ArmConstants.minAngleInDegrees, "max", ArmConstants.maxAngleInDegrees))
        .withSize(2, 1)
        .withPosition(colIndex, rowIndex++);

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

    ShuffleboardLayout aimingModeLayout =
        driverTab
            .getLayout("Aiming Mode", BuiltInLayouts.kGrid)
            //            .withProperties(Map.of("Label position", "HIDDEN"))
            .withSize(2, 3)
            .withPosition(colIndex, rowIndex);
    rowIndex += 3;

    ampModeCommand.setName("Amp Mode");
    speakerModeCommand.setName("Speaker Mode");
    speakerModeFromSubwoofer.setName("Speaker (Sub)");
    speakerModeFromPodium.setName("Speaker (Pod)");
    speakerModeVisionBased.setName("Speaker (Auto)");

    aimingModeLayout
        .addString("Aiming Mode", () -> DevilBotState.getShootingModeName())
        .withWidget(BuiltInWidgets.kTextView)
        .withPosition(0, 0)
        .withSize(2, 2);

    aimingModeLayout.add(ampModeCommand).withPosition(0, 1);
    aimingModeLayout.add(speakerModeFromSubwoofer).withPosition(0, 2);
    aimingModeLayout.add(speakerModeFromPodium).withPosition(0, 3);
    aimingModeLayout.add(speakerModeVisionBased).withPosition(0, 4);

    Mechanism2d mech2d = new Mechanism2d(60, 60);
    RobotConfig.arm.add2dSim(mech2d);
    RobotConfig.intake.add2dSim(mech2d);
    RobotConfig.shooter.add2dSim(mech2d);
    RobotConfig.climber.add2dSim(mech2d);
    RobotConfig.led.add2dSim(mech2d);
    ;

    SmartDashboard.putData("Inferno 2D Simulation", mech2d);
  }

  private static void setupCommonControls(CommandXboxController controller) {
    /* Climber Controls */
    /*     Y Button = Prepare to Climb
     *     X Button = Climber
     */
    controller.y().onTrue(RobotConfig.intake.getTurnOffCommand());
    controller.y().onTrue(RobotConfig.shooter.getTurnOffCommand());
    controller.y().onTrue(RobotConfig.arm.getStowCommand());
    controller.y().onTrue(RobotConfig.climber.getExtendCommand());

    controller.x().onTrue(RobotConfig.climber.getRetractCommand());

    /* Target Selection Controls */
    /*     A Button = Amp Mode
     *     B Button = Speaker Mode
     */
    controller.a().onTrue(ampModeCommand);
    controller.b().onTrue(speakerModeCommand);
  }

  public static boolean driveOverride(CommandXboxController mainController) {
    return (Math.abs(mainController.getLeftY()) > 0.05)
        || (Math.abs(mainController.getLeftX()) > 0.05)
        || (Math.abs(mainController.getRightY()) > 0.05)
        || (Math.abs(mainController.getRightX()) > 0.05);
  }

  public static void setupMainControls(CommandXboxController mainController) {
    /**** Drive Controls ****/
    /* Competition:
     *    Left Stick Up/Down/Left/Right = Robot Strafe
     *    Right Stick Left/Right = Robot Rotate
     */
    DevilBotState.setDriveMode(DriveMode.FIELD);

    RobotConfig.drive.setDefaultCommand(
        new DriveCommand(
            RobotConfig.drive,
            () ->
                MathUtil.applyDeadband(-mainController.getLeftY(), 0.05), // Robot Strafe Front/Back
            () ->
                MathUtil.applyDeadband(-mainController.getLeftX(), 0.05), // Robot Strafe Left/Right
            () -> MathUtil.applyDeadband(-mainController.getRightX(), 0.05))); // Robot Rotate

    /* Debug/Test Only:
     *    Back Button = Zero Pose
     *    Start Button = Toggle Drive Orientation
     */
    mainController.back().onTrue(new InstantCommand(() -> RobotConfig.drive.resetOdometry()));
    mainController
        .start()
        .onTrue(
            new InstantCommand(
                () -> {
                  if (DevilBotState.isFieldOriented()) {
                    DevilBotState.setDriveMode(DriveMode.ROBOT);
                  } else {
                    DevilBotState.setDriveMode(DriveMode.FIELD);
                  }
                })); // Toggle Drive Orientation

    /**** Note Manipulation Controls ****/
    /* Competition:
     *    Left Trigger =
     *       On True:
     *           Piece Not Detected: "Intake Note"
     *           Piece Detected: "Stow Note"
     *       On False: "Stow Arm"
     *    Left Bumper = "Eject Note"
     *    Right Bumper = "Aim"
     *    Right Trigger = "Shoot Note"
     */
    mainController
        .leftTrigger()
        .onTrue(
            new InstantCommand(
                () -> {
                  if ((false == RobotConfig.intake.isPieceDetected())
                      || (false == DevilBotState.isPieceDetectionEnabled())) {
                    RobotConfig.intake.turnOn();
                    RobotConfig.arm.setAngle(ArmConstants.intakeAngleInDegrees); // Intake Note
                  } else {
                    RobotConfig.arm.setAngle(ArmConstants.stowIntakeAngleInDegrees); // Stow Arm
                  }
                },
                RobotConfig.arm));

    mainController
        .leftTrigger()
        .onFalse(
            new InstantCommand(
                () -> RobotConfig.arm.setAngle(ArmConstants.stowIntakeAngleInDegrees),
                RobotConfig.arm)); // Stow Arm

    mainController
        .leftBumper()
        .onTrue(
            new EjectPiece(RobotConfig.intake, RobotConfig.arm, RobotConfig.shooter)); // Eject Note

    mainController.rightBumper().onTrue(RobotConfig.intake.getTurnOffCommand());

    mainController
        .rightBumper()
        .onTrue(
            new SelectCommand<>(
                    Map.ofEntries(
                        Map.entry(
                            true,
                            AutoBuilder.pathfindToPoseFlipped(
                                new Pose2d(1.8, 7.75, Rotation2d.fromDegrees(-90)),
                                new PathConstraints(3.0, 2.0, 2 * Math.PI, 3 * Math.PI))),
                        Map.entry(
                            false,
                            new DriveToYaw(
                                    RobotConfig.drive,
                                    () -> DevilBotState.getVisionRobotYawToTarget())
                                .withTimeout(DriveConstants.pidTimeoutInSeconds))),
                    () -> DevilBotState.isAmpMode())
                .until(() -> driveOverride(mainController)));

    mainController
        .rightBumper()
        .onTrue(
            new SetShooterVelocity(RobotConfig.shooter, () -> DevilBotState.getShooterVelocity())
                .withTimeout(ShooterConstants.pidTimeoutInSeconds) // turn on shooter
            );

    mainController
        .rightBumper()
        .onTrue(
            new InstantCommand(
                    () -> {
                      Optional<Double> armAngle = DevilBotState.getArmAngleToTarget();
                      if (armAngle.isPresent()) {
                        RobotConfig.arm.setAngle((armAngle.get()));
                      }
                    },
                    RobotConfig.arm)
                .onlyIf(() -> !DevilBotState.isAmpMode()));

    mainController
        .rightTrigger()
        .onTrue(
            new SequentialCommandGroup(
                new InstantCommand(
                    () -> RobotConfig.drive.lockPose(),
                    RobotConfig.drive), // lock drive wheels/pose
                new SetShooterVelocity(
                        RobotConfig.shooter, () -> DevilBotState.getShooterVelocity())
                    .withTimeout(
                        ShooterConstants
                            .pidTimeoutInSeconds), // set shooter velocity in case it's not already
                // on
                RobotConfig.intake.getTurnOnCommand())); // Shoot Note

    // Trigger rumble when a note is detected
    Trigger noteDetectedTrigger = new Trigger(() -> RobotConfig.intake.isPieceDetected());
    noteDetectedTrigger.onTrue(
        new SequentialCommandGroup(
            // Starts the controller rumble
            new InstantCommand(() -> mainController.getHID().setRumble(RumbleType.kBothRumble, 1)),
            // Rumbles for 2 seconds
            new WaitCommand(2),
            // Ends the controller rumble
            new InstantCommand(
                () -> mainController.getHID().setRumble(RumbleType.kBothRumble, 0))));

    // Trigger rumble when Shooter at RPM setpoint
    Trigger shooterRPMTrigger =
        new Trigger(
            () ->
                Units.radiansPerSecondToRotationsPerMinute(RobotConfig.shooter.getCurrentSpeed())
                        >= DevilBotState.getShooterVelocity()
                            - ShooterConstants.pidVelocityErrorInRPM
                    && Units.radiansPerSecondToRotationsPerMinute(
                            RobotConfig.shooter.getCurrentSpeed())
                        <= DevilBotState.getShooterVelocity()
                            + ShooterConstants.pidVelocityErrorInRPM);
    shooterRPMTrigger.onTrue(
        new SequentialCommandGroup(
            // Starts the controller rumble
            new InstantCommand(() -> mainController.getHID().setRumble(RumbleType.kBothRumble, 1)),
            // Rumbles for 2 seconds
            new WaitCommand(2),
            // Ends the controller rumble
            new InstantCommand(
                () -> mainController.getHID().setRumble(RumbleType.kBothRumble, 0))));

    EventLoop eventLoop = CommandScheduler.getInstance().getDefaultButtonLoop();
    BooleanEvent havePiece =
        new BooleanEvent(
            eventLoop,
            () -> RobotConfig.intake.isPieceDetected() && DevilBotState.isPieceDetectionEnabled());

    Trigger havePieceTriggerRising = havePiece.rising().castTo(Trigger::new);

    // We've picked up a note
    havePieceTriggerRising
        .and(() -> DevilBotState.getState() == State.TELEOP)
        .onTrue(
            new SequentialCommandGroup(
                RobotConfig.intake.getTurnOffCommand(), // Turn off intake
                new InstantCommand(
                    () -> RobotConfig.arm.setAngle(ArmConstants.stowIntakeAngleInDegrees),
                    RobotConfig.arm), // Put arm in stow mode
                new InstantCommand(
                    () -> RobotConfig.shooter.runVelocity(DevilBotState.getShooterVelocity()),
                    RobotConfig.shooter) // Turn on the shooter
                ));

    Trigger havePieceTriggerFalling = havePiece.falling().castTo(Trigger::new);

    // We no longer have a note
    havePieceTriggerFalling
        .and(() -> DevilBotState.getState() == State.TELEOP)
        .onFalse(
            new ParallelCommandGroup(
                RobotConfig.intake.getTurnOnCommand(), // Turn on intake
                new InstantCommand(
                    () -> RobotConfig.arm.setAngle(ArmConstants.stowIntakeAngleInDegrees),
                    RobotConfig.arm), // Put arm in stow mode
                new InstantCommand(
                    () -> RobotConfig.shooter.runVelocity(0),
                    RobotConfig.shooter) // Turn off shooter
                ));

    BooleanEvent stateChangedEvent =
        new BooleanEvent(
            eventLoop,
            () -> (DevilBotState.stateChanged()) && DevilBotState.getState() != State.AUTO);

    Trigger stateChangedEventTrigger = stateChangedEvent.rising().castTo(Trigger::new);
    stateChangedEventTrigger.onTrue(getRobotStateTransitionCommand());
  }

  public static Command getResetDisableAllSubsystemsCommand() {
    return new ParallelCommandGroup(
        getRobotStateTransitionCommand(),
        new InstantCommand(
            () -> {
              RobotConfig.arm.runVoltage(0);
            }));
  }

  private static Command getRobotStateTransitionCommand() {
    return new ParallelCommandGroup(
        RobotConfig.shooter.getTurnOffCommand(),
        RobotConfig.intake.getTurnOffCommand(),
        new InstantCommand(
            () -> {
              mainController.getHID().setRumble(RumbleType.kBothRumble, 0);
              secondaryController.getHID().setRumble(RumbleType.kBothRumble, 0);
            }));
  }

  private static void setupSecondaryControls(CommandXboxController controller) {
    /* Low Level Intake Controls
     *    Left Bumper = Intake: Out
     *    Right Bumper = Intake: In
     *    D-Pad Down = Intake: Off
     *    Back Button = Intake: Disable Detection
     *    Start Button = Intake: Enable Detection
     */
    controller
        .leftBumper()
        .onTrue(
            new InstantCommand(
                () -> RobotConfig.intake.runVoltage(-IntakeConstants.defaultSpeedInVolts),
                RobotConfig.intake)); // Intake: Out
    controller
        .rightBumper()
        .onTrue(
            new InstantCommand(
                () -> RobotConfig.intake.runVoltage(IntakeConstants.defaultSpeedInVolts),
                RobotConfig.intake)); // Intake: In

    controller
        .rightTrigger()
        .onTrue(
            new ParallelCommandGroup(
                RobotConfig.intake.getTurnOffCommand(),
                new SetShooterVelocity(
                        RobotConfig.shooter, () -> DevilBotState.getShooterVelocity())
                    .withTimeout(ShooterConstants.pidTimeoutInSeconds), // turn on shooter
                /* TODO: Use ArmToPositionTP instead of setting arm angle directly */
                new InstantCommand(
                    () -> {
                      if (DevilBotState.isAmpMode()) {
                        RobotConfig.arm.setAngle(ArmConstants.ampScoreAngleInDegrees);
                      } else {
                        Optional<Double> armAngle = DevilBotState.getArmAngleToTarget();
                        if (armAngle.isPresent()) {
                          RobotConfig.arm.setAngle((armAngle.get()));
                        }
                      }
                    },
                    RobotConfig.arm) // adjust arm angle based on vision's distance from target
                )); // Aim

    controller.pov(180).onTrue(RobotConfig.intake.getTurnOffCommand()); // Intake: Off
    controller
        .back()
        .onTrue(
            new InstantCommand(
                () ->
                    DevilBotState.setPieceDetectionMode(
                        PieceDetectionMode.DISABLED))); // Intake: Disable Detection
    controller
        .start()
        .onTrue(
            new InstantCommand(
                () ->
                    DevilBotState.setPieceDetectionMode(
                        PieceDetectionMode.ENABLED))); // Intake: Enable Detection

    /* Low Level Arm Controls */
    /* Left Stick Up/Down = Arm Down/Up */
    EventLoop eventLoop = CommandScheduler.getInstance().getDefaultButtonLoop();
    BooleanEvent leftYPressed =
        new BooleanEvent(eventLoop, () -> Math.abs(controller.getLeftY()) > 0.05);

    Trigger leftYPressedTrigger = leftYPressed.castTo(Trigger::new);
    leftYPressedTrigger.whileTrue(
        new ArmCommand(
            RobotConfig.arm,
            () -> MathUtil.applyDeadband(-controller.getLeftY(), 0.05))); //  Arm Up/Down

    /* Aim Controls
     * D-Pad Up = Aim:Auto
     * D-Pad Left = Aim:Subwoofer
     * D-Pad Right = Aim:Podium
     */
    controller.pov(0).onTrue(speakerModeVisionBased);
    controller.pov(90).onTrue(speakerModeFromPodium);
    controller.pov(270).onTrue(speakerModeFromSubwoofer);
  }

  private static void setupLedControls() {
    Trigger noteDetectedTrigger = new Trigger(() -> RobotConfig.intake.isPieceDetected());
    noteDetectedTrigger.onTrue(RobotConfig.led.getNoteDetectionCommand());

    Trigger AmpModeTrigger = new Trigger(() -> DevilBotState.isAmpMode());
    AmpModeTrigger.onTrue(RobotConfig.led.getAmpModeCommand());
    AmpModeTrigger.onFalse(RobotConfig.led.getSpeakerModeCommand());
  }
}
