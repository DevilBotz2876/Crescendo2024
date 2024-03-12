package frc.robot.controls;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.commands.auto.AutoPrepareForIntake;
import frc.robot.commands.auto.AutoScorePiece;
import frc.robot.commands.debug.PrepareForScore;
import frc.robot.commands.debug.TestShooterAngle;
import frc.robot.commands.vision.AlignToTarget;
import frc.robot.config.RobotConfig;
import frc.robot.config.RobotConfig.ArmConstants;
import frc.robot.config.RobotConfig.IntakeConstants;
import frc.robot.config.RobotConfig.ShooterConstants;
import frc.robot.util.DevilBotState;
import java.util.Map;

public class DebugControls {
  public static void setupControls() {
    int colIndex = 0;
    int rowIndex = 0;
    ShuffleboardTab debugTab = Shuffleboard.getTab("Debug");

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

    colIndex += 2;
    rowIndex = 0;
    GenericEntry intakeFeedVoltageEntry =
        debugTab
            .add("Intake: Feed Volts", IntakeConstants.feedSpeedInVolts)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", 0, "max", 12))
            .withPosition(colIndex, rowIndex++)
            .withSize(2, 1)
            .getEntry();

    debugTab
        .add(
            "Assist: Shoot Piece",
            new AutoScorePiece(
                RobotConfig.intake,
                RobotConfig.shooter,
                () -> intakeFeedVoltageEntry.getDouble(IntakeConstants.feedSpeedInVolts)))
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
                () -> intakeFeedVoltageEntry.getDouble(IntakeConstants.feedSpeedInVolts),
                () -> armAngleEntry.getDouble(ArmConstants.subwooferScoreAngleInDegrees)))
        .withPosition(colIndex, rowIndex++)
        .withSize(2, 1);

    GenericEntry visionTargetId =
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
            new AlignToTarget(
                RobotConfig.drive,
                RobotConfig.vision,
                () -> (int) visionTargetId.getInteger(DevilBotState.getActiveTargetId())))
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
