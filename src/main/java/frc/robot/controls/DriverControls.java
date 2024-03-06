package frc.robot.controls;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.assist.EjectPiece;
import frc.robot.commands.assist.PrepareForIntake;
import frc.robot.commands.assist.ScorePiece;
import frc.robot.commands.climber.ClimberCommand;
import frc.robot.commands.drive.DriveCommand;
import frc.robot.commands.shooter.SetShooterVelocity;
import frc.robot.config.RobotConfig;
import frc.robot.config.RobotConfig.ArmConstants;
import frc.robot.config.RobotConfig.ShooterConstants;
import frc.robot.util.RobotState;
import frc.robot.util.RobotState.DriveMode;
import frc.robot.util.RobotState.TargetMode;
import java.util.Map;
import java.util.Optional;

public class DriverControls {
  private static GenericEntry ampModeEntry;
  private static GenericEntry fieldOrientedModeEntry;

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

    ampModeEntry =
        driverTab
            .add("Amp Mode", RobotState.isAmpMode())
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withPosition(colIndex, rowIndex++)
            .withSize(2, 1)
            .getEntry();

    fieldOrientedModeEntry =
        driverTab
            .add("Field Oriented", RobotState.isFieldOriented())
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withPosition(colIndex, rowIndex++)
            .withSize(2, 1)
            .getEntry();

    /* TODO: Intake Camera */

    /* Shooter Camera */
    colIndex += 2;
    rowIndex = 0;
    driverTab
        .addCamera("Shooter Camera", "shooter", "mjpg:http://10.28.76.11:1181/?action=stream")
        .withProperties(Map.of("showControls", false))
        .withPosition(colIndex, rowIndex)
        .withSize(3, 3);
    rowIndex += 3;

    Shuffleboard.selectTab("Driver");
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
    fieldOrientedModeEntry.setBoolean(RobotState.isFieldOriented());

    RobotConfig.drive.setDefaultCommand(
        new DriveCommand(
            RobotConfig.drive,
            () -> MathUtil.applyDeadband(-mainController.getLeftY(), 0.05),
            () -> MathUtil.applyDeadband(-mainController.getLeftX(), 0.05),
            () -> MathUtil.applyDeadband(-mainController.getRightX(), 0.05)));

    // TODO: Eventually remove!....this is for debug only
    mainController.back().onTrue(new InstantCommand(() -> RobotConfig.drive.resetOdometry()));

    /* Climber Controls */
    /*     D-Pad Up = Climber Up
     *     D-Pad Down = Climber Down
     */
    mainController.pov(0).onTrue(new ClimberCommand(RobotConfig.climber, true));
    mainController.pov(180).onTrue(new ClimberCommand(RobotConfig.climber, false));

    /* Driver Assist Controls */
    /*     Left Trigger = "Prepare for Intake"
     *     Right Trigger = "Score Piece"
     *     Left Bumper = "Eject Piece"
     *     Right Bumper = "Prepare for Score" aka Auto Orient
     */
    mainController
        .leftTrigger()
        .onTrue(
            new SequentialCommandGroup(
                new InstantCommand(
                    () -> RobotConfig.shooter.runVelocity(0),
                    RobotConfig.shooter), // Turn off shooter in case it is on
                new PrepareForIntake(
                    RobotConfig.arm, RobotConfig.intake))); // Set arm angle and turn on intake
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
                new ScorePiece(
                    RobotConfig.intake,
                    RobotConfig.shooter))); // Turn on intake to feed piece into shooter
    mainController.leftBumper().onTrue(new EjectPiece(RobotConfig.intake, RobotConfig.arm));

    /* TODO: Make this is a toggle, not just while right bumper is pressed */
    mainController
        .rightBumper()
        .whileTrue(
            new ParallelCommandGroup(
                new InstantCommand(
                    () -> RobotConfig.intake.runVoltage(0),
                    RobotConfig.intake), // turn off intake in case it is on
                new DriveCommand(
                    RobotConfig.drive,
                    () -> MathUtil.applyDeadband(-mainController.getLeftY(), 0.05),
                    () -> MathUtil.applyDeadband(-mainController.getLeftX(), 0.05),
                    () -> MathUtil.applyDeadband(-mainController.getRightX(), 0.05),
                    () ->
                        RobotConfig.vision.getYawToAprilTag(
                            RobotState
                                .getActiveTargetId())), // adjust robot yaw to line up with vision
                // target
                new SetShooterVelocity(RobotConfig.shooter, () -> RobotState.getShooterVelocity())
                    .withTimeout(ShooterConstants.pidTimeoutInSeconds), // turn on shooter
                /* TODO: Use ArmToPositionTP instead of setting arm angle directly */
                new InstantCommand(
                        () -> {
                          if (RobotState.isAmpMode()) {
                            RobotConfig.arm.setAngle(ArmConstants.ampScoreAngleInDegrees);
                          } else {
                            Optional<Double> distanceToTarget =
                                RobotConfig.vision.getDistanceToAprilTag(
                                    RobotState.getActiveTargetId());
                            if (distanceToTarget.isPresent()) {
                              Optional<Double> armAngle =
                                  RobotConfig.instance.getArmAngleFromDistance(
                                      distanceToTarget.get());
                              if (armAngle.isPresent()) {
                                RobotConfig.arm.setAngle((armAngle.get()));
                              }
                            }
                          }
                        },
                        RobotConfig.arm) // adjust arm angle based on vision's distance from target
                    .repeatedly()));

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
                  ampModeEntry.setBoolean(RobotState.isAmpMode());
                }));
    mainController
        .b()
        .onTrue(
            new InstantCommand(
                () -> {
                  RobotState.setTargetMode(TargetMode.SPEAKER);
                  ampModeEntry.setBoolean(RobotState.isAmpMode());
                }));
  }
}
