package frc.robot.controls;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.commands.assist.PrepareForIntake;
import frc.robot.commands.assist.PrepareForScore;
import frc.robot.commands.assist.ScorePiece;
import frc.robot.commands.debug.TestShooterAngle;
import frc.robot.commands.vision.AlignToTarget;
import frc.robot.config.RobotConfig;
import frc.robot.config.RobotConfig.ArmConstants;
import frc.robot.config.RobotConfig.IntakeConstants;
import frc.robot.config.RobotConfig.ShooterConstants;
import frc.robot.util.RobotState;
import java.util.Map;

public class DebugControls {
  public static void setupGUI() {
    int colIndex = 0;
    int rowIndex = 0;
    ShuffleboardTab assistTab = Shuffleboard.getTab("Debug");

    assistTab
        .add(
            "Assist: Prepare For Intake", new PrepareForIntake(RobotConfig.arm, RobotConfig.intake))
        .withPosition(colIndex, rowIndex++)
        .withSize(2, 1);
    assistTab
        .add("Intake: Angle", ArmConstants.minAngleInDegrees)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(
            Map.of("min", ArmConstants.minAngleInDegrees, "max", ArmConstants.maxAngleInDegrees))
        .withPosition(colIndex, rowIndex++)
        .withSize(2, 1);

    assistTab
        .add("Intake: Index Volts", IntakeConstants.indexSpeedInVolts)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", 0, "max", 12))
        .withPosition(colIndex, rowIndex++)
        .withSize(2, 1);

    colIndex += 2;
    rowIndex = 0;
    assistTab
        .add(
            "Assist: Prepare For Score",
            new PrepareForScore(RobotConfig.arm, RobotConfig.shooter, () -> RobotState.isAmpMode()))
        .withPosition(colIndex, rowIndex++)
        .withSize(2, 1);
    assistTab
        .add("Shooter: Velocity", ShooterConstants.velocityInRPMs)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", 0, "max", 6000))
        .withPosition(colIndex, rowIndex++)
        .withSize(2, 1);
    assistTab
        .add("Shooter: Angle", ArmConstants.subwooferScoreAngleInDegrees)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(
            Map.of("min", ArmConstants.minAngleInDegrees, "max", ArmConstants.maxAngleInDegrees))
        .withPosition(colIndex, rowIndex++)
        .withSize(2, 1);
    assistTab
        .add("Shooter: Velocity (Amp)", ShooterConstants.ampScoreVelocityInRPMs)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", 0, "max", 6000))
        .withPosition(colIndex, rowIndex++)
        .withSize(2, 1);
    assistTab
        .add("Shooter: Angle (Amp)", ArmConstants.ampScoreAngleInDegrees)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(
            Map.of("min", ArmConstants.minAngleInDegrees, "max", ArmConstants.maxAngleInDegrees))
        .withPosition(colIndex, rowIndex++)
        .withSize(2, 1);

    assistTab
        .add("Amp Mode", RobotState.isAmpMode())
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withPosition(colIndex, rowIndex++)
        .withSize(2, 1)
        .getEntry();

    colIndex += 2;
    rowIndex = 0;
    assistTab
        .add("Assist: Shoot Piece", new ScorePiece(RobotConfig.intake, RobotConfig.shooter))
        .withPosition(colIndex, rowIndex++)
        .withSize(2, 1);
    assistTab
        .add("Intake: Feed Volts", IntakeConstants.feedSpeedInVolts)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", 0, "max", 12))
        .withPosition(colIndex, rowIndex++)
        .withSize(2, 1);

    colIndex += 2;
    rowIndex = 0;
    assistTab
        .add(
            "Test: Shooter Angle",
            new TestShooterAngle(RobotConfig.shooter, RobotConfig.intake, RobotConfig.arm))
        .withPosition(colIndex, rowIndex++)
        .withSize(2, 1);

    assistTab
        .add("Vision: Target ID", IntakeConstants.feedSpeedInVolts)
        .withWidget(BuiltInWidgets.kTextView)
        .withProperties(Map.of("min", 1, "max", 16))
        .withPosition(colIndex, rowIndex++)
        .withSize(2, 1);
    NetworkTable assistGUI = NetworkTableInstance.getDefault().getTable("Shuffleboard/Debug");

    assistTab
        .add(
            "Vision: Align To Target",
            new AlignToTarget(
                RobotConfig.drive,
                RobotConfig.vision,
                () ->
                    (int)
                        assistGUI
                            .getEntry("Vision: Target ID")
                            .getInteger(RobotState.getActiveTargetId())))
        .withPosition(colIndex, rowIndex++)
        .withSize(2, 1);

    assistTab
        .add("Field Oriented Drive", RobotConfig.drive.isFieldOrientedDrive())
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withPosition(colIndex, rowIndex++)
        .withSize(2, 1)
        .getEntry();

    assistTab
        .add("Drive Speed Limit", 100)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", 0, "max", 100))
        .getEntry();

    assistTab
        .add("Drive Turn Limit", 100)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", 0, "max", 100))
        .getEntry();

    SendableChooser<String> driveSpeedChooser = new SendableChooser<>();
    driveSpeedChooser.addOption("Linear Mode", "Linear Mode");
    driveSpeedChooser.setDefaultOption("Squared Mode", "Squared Mode");
    driveSpeedChooser.addOption("Cubed Mode", "Cubed Mode");
    assistTab.add("Drive Response Curve", driveSpeedChooser);
  }
}
