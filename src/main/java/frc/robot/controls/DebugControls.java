package frc.robot.controls;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.arm.ArmToPosition;
import frc.robot.commands.auto.AutoPrepareForIntake;
import frc.robot.commands.auto.AutoScorePiece;
import frc.robot.commands.debug.PrepareForScore;
import frc.robot.commands.debug.TestShooterAngle;
import frc.robot.commands.drive.DriveToYaw;
import frc.robot.config.RobotConfig;
import frc.robot.config.RobotConfig.ArmConstants;
import frc.robot.config.RobotConfig.DriveConstants;
import frc.robot.config.RobotConfig.IntakeConstants;
import frc.robot.config.RobotConfig.ShooterConstants;
import frc.robot.util.DevilBotState;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Optional;

public class DebugControls {
  private static int setupshooterCalibrationControls(
      ShuffleboardTab tab, int col, int row, int maxWidth) {
    int layoutColIndex = 0;
    int layoutRowIndex = 0;
    int layoutMaxHeight = 4;
    List<Command> commands = new ArrayList<Command>();
    maxWidth = Math.min(2, maxWidth);

    /* shooterCalibration Controls */
    ShuffleboardLayout shooterCalibrationLayout =
        tab.getLayout("shooterCalibration", BuiltInLayouts.kGrid)
            .withProperties(Map.of("Label position", "TOP", "Number of columns", 1))
            .withSize(maxWidth, layoutMaxHeight)
            .withPosition(col, row);
    row += layoutMaxHeight;

    ShuffleboardLayout shooterCalibrationStatusLayout =
        shooterCalibrationLayout
            .getLayout("Status", BuiltInLayouts.kGrid)
            .withProperties(Map.of("Number of columns", 1))
            .withPosition(0, 0);
    layoutColIndex = 0;
    layoutRowIndex = 0;

    shooterCalibrationStatusLayout
        .addDouble("Odometry Distance (meters)", () -> RobotConfig.drive.getPose().getX())
        .withWidget(BuiltInWidgets.kTextView)
        .withPosition(layoutColIndex, layoutRowIndex++);

    shooterCalibrationStatusLayout
        .addDouble(
            "Vision Distance (meters)",
            () -> {
              Optional<Double> distance =
                  RobotConfig.vision.getDistanceToAprilTag(DevilBotState.getActiveTargetId());
              if (distance.isPresent()) return distance.get();
              else return -1;
            })
        .withWidget(BuiltInWidgets.kTextView)
        .withPosition(layoutColIndex, layoutRowIndex++);

    shooterCalibrationStatusLayout
        .addDouble("Arm Abs Angle (degrees)", () -> RobotConfig.arm.getAngle())
        .withWidget(BuiltInWidgets.kTextView)
        .withProperties(
            Map.of("min", ArmConstants.minAngleInDegrees, "max", ArmConstants.maxAngleInDegrees))
        .withSize(2, 1)
        .withPosition(layoutColIndex, layoutRowIndex++);

    shooterCalibrationStatusLayout
        .addDouble(
            "Shooter Velocity (RPMs)",
            () -> Units.radiansPerSecondToRotationsPerMinute(RobotConfig.shooter.getCurrentSpeed()))
        .withWidget(BuiltInWidgets.kTextView)
        .withPosition(layoutColIndex, layoutRowIndex++);
    /*
        ShuffleboardLayout shooterCalibrationCommandLayout =
            shooterCalibrationLayout
                .getLayout("Commands", BuiltInLayouts.kGrid)
                .withProperties(Map.of("Label position", "HIDDEN", "Number of columns", 1))
                .withPosition(0, 1);
        layoutColIndex = 0;
        layoutRowIndex = 0;
    */
    return maxWidth;
  }

  public static void setupControls() {
    int colIndex = 0;
    int rowIndex = 0;
    ShuffleboardTab debugTab = Shuffleboard.getTab("Debug");

    setupshooterCalibrationControls(debugTab, colIndex, rowIndex, 2);
    colIndex += 2;

    GenericEntry intakeAngleEntry =
        debugTab
            .add("Intake: Angle", ArmConstants.intakeAngleInDegrees)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(
                Map.of(
                    "min", ArmConstants.minAngleInDegrees, "max", ArmConstants.maxAngleInDegrees))
            .withPosition(colIndex, rowIndex++)
            .withSize(2, 1)
            .getEntry();

    GenericEntry intakeVoltageEntry =
        debugTab
            .add("Intake: Volts", IntakeConstants.defaultSpeedInVolts)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", 0, "max", 12))
            .withPosition(colIndex, rowIndex++)
            .withSize(2, 1)
            .getEntry();

    debugTab
        .add(
            "Assist: Prepare For Intake",
            new AutoPrepareForIntake(
                RobotConfig.arm,
                RobotConfig.intake,
                () -> intakeAngleEntry.getDouble(ArmConstants.intakeAngleInDegrees),
                () -> intakeVoltageEntry.getDouble(IntakeConstants.defaultSpeedInVolts)))
        .withPosition(colIndex, rowIndex++)
        .withSize(2, 1);

    colIndex += 2;
    rowIndex = 0;
    GenericEntry shooterVelocityEntry =
        debugTab
            .add("Shooter: Velocity", ShooterConstants.velocityInRPMs)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", 0, "max", 6000))
            .withPosition(colIndex, rowIndex++)
            .withSize(2, 1)
            .getEntry();
    GenericEntry armAngleEntry =
        debugTab
            .add("Shooter: Angle", ArmConstants.subwooferScoreAngleInDegrees)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(
                Map.of(
                    "min", ArmConstants.minAngleInDegrees, "max", ArmConstants.maxAngleInDegrees))
            .withPosition(colIndex, rowIndex++)
            .withSize(2, 1)
            .getEntry();
    GenericEntry shooterVelocityAmpEntry =
        debugTab
            .add("Shooter: Velocity (Amp)", ShooterConstants.ampScoreVelocityInRPMs)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", 0, "max", 6000))
            .withPosition(colIndex, rowIndex++)
            .withSize(2, 1)
            .getEntry();
    GenericEntry armAngleAmpEntry =
        debugTab
            .add("Shooter: Angle (Amp)", ArmConstants.ampScoreAngleInDegrees)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(
                Map.of(
                    "min", ArmConstants.minAngleInDegrees, "max", ArmConstants.maxAngleInDegrees))
            .withPosition(colIndex, rowIndex++)
            .withSize(2, 1)
            .getEntry();

    debugTab
        .add(
            "Assist: Prepare For Score",
            new PrepareForScore(
                RobotConfig.arm,
                RobotConfig.shooter,
                () -> {
                  if (DevilBotState.isAmpMode())
                    return armAngleAmpEntry.getDouble(ArmConstants.ampScoreAngleInDegrees);
                  else return armAngleEntry.getDouble(ArmConstants.subwooferScoreAngleInDegrees);
                },
                () -> {
                  if (DevilBotState.isAmpMode())
                    return shooterVelocityAmpEntry.getDouble(
                        ShooterConstants.ampScoreVelocityInRPMs);
                  else return shooterVelocityEntry.getDouble(ShooterConstants.velocityInRPMs);
                }))
        .withPosition(colIndex, rowIndex++)
        .withSize(2, 1);

    debugTab
        .add(
            "Assist: Shoot Piece",
            new AutoScorePiece(
                RobotConfig.intake,
                RobotConfig.shooter,
                () -> intakeVoltageEntry.getDouble(IntakeConstants.defaultSpeedInVolts)))
        .withPosition(colIndex, rowIndex++)
        .withSize(2, 1);

    colIndex += 2;
    rowIndex = 0;

    debugTab
        .add(
            "Test: Shooter Angle",
            new TestShooterAngle(
                RobotConfig.shooter,
                RobotConfig.intake,
                RobotConfig.arm,
                () -> shooterVelocityEntry.getDouble(ShooterConstants.velocityInRPMs),
                () -> intakeVoltageEntry.getDouble(IntakeConstants.defaultSpeedInVolts),
                () -> armAngleEntry.getDouble(ArmConstants.subwooferScoreAngleInDegrees)))
        .withPosition(colIndex, rowIndex++)
        .withSize(2, 1);

    debugTab
        .add(
            "Arm To Position",
            new ArmToPosition(
                RobotConfig.arm,
                () -> armAngleEntry.getDouble(ArmConstants.subwooferScoreAngleInDegrees)))
        .withPosition(colIndex, rowIndex++)
        .withSize(2, 1);

    debugTab
        .add("Vision: Target ID", DevilBotState.getActiveTargetId())
        .withWidget(BuiltInWidgets.kTextView)
        .withProperties(Map.of("min", 1, "max", 16))
        .withPosition(colIndex, rowIndex++)
        .withSize(2, 1)
        .getEntry();

    debugTab
        .add(
            "Vision: Align To Target",
            new DriveToYaw(RobotConfig.drive, () -> DevilBotState.getVisionRobotYawToTarget())
                .withTimeout(DriveConstants.pidTimeoutInSeconds))
        .withPosition(colIndex, rowIndex++)
        .withSize(2, 1);

    debugTab.add(RobotConfig.drive).withPosition(colIndex, rowIndex++).withSize(2, 1);
    /*
    debugTab
        .add("Drive Speed Limit", 100)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", 0, "max", 100))
        .getEntry();

    debugTab
        .add("Drive Turn Limit", 100)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", 0, "max", 100))
        .getEntry();

    SendableChooser<String> driveSpeedChooser = new SendableChooser<>();
    driveSpeedChooser.addOption("Linear Mode", "Linear Mode");
    driveSpeedChooser.setDefaultOption("Squared Mode", "Squared Mode");
    driveSpeedChooser.addOption("Cubed Mode", "Cubed Mode");
    debugTab.add("Drive Response Curve", driveSpeedChooser);
     */

    /*

    ShuffleboardTab armTab;
    GenericEntry armVoltsEntry;
    GenericEntry armDegreesEntry;
    GenericEntry highLimitEntry;
    GenericEntry lowLimitEntry;


      // create arm tab on ShuffleBoard
      armTab = Shuffleboard.getTab("Arm");
      // Create volt entry under arm tab as a number sider with min = -4 and max = 4
      armVoltsEntry =
          armTab
              .add("Volts", 0)
              .withWidget(BuiltInWidgets.kNumberSlider)
              .withProperties(Map.of("min", -4, "max", 4))
              .getEntry();

      highLimitEntry =
          armTab.add("HighLimit", false).withWidget(BuiltInWidgets.kBooleanBox).getEntry();

      lowLimitEntry = armTab.add("LowLimit", false).withWidget(BuiltInWidgets.kBooleanBox).getEntry();

      armTab
          .add("Degrees Setpoint", 0.0)
          .withWidget(BuiltInWidgets.kNumberSlider)
          .withProperties(
              Map.of("min", ArmConstants.minAngleInDegrees, "max", ArmConstants.maxAngleInDegrees))
          .getEntry();

        highLimitEntry.setBoolean(true);
        lowLimitEntry.setBoolean(true);

       */
  }
}
