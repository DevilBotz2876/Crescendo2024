package frc.robot.controls;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.arm.ArmToPosition;
import frc.robot.commands.debug.TestShooterAngle;
import frc.robot.commands.shooter.SetShooterVelocity;
import frc.robot.config.RobotConfig;
import frc.robot.config.RobotConfig.ArmConstants;
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
        tab.getLayout("Shooter Calibration", BuiltInLayouts.kGrid)
            .withProperties(Map.of("Label position", "LEFT", "Number of columns", 1))
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

    ShuffleboardLayout shooterCalibrationCommandLayout =
        shooterCalibrationLayout
            .getLayout("Commands", BuiltInLayouts.kGrid)
            .withProperties(Map.of("Label position", "HIDDEN", "Number of columns", 1))
            .withPosition(0, 1);
    layoutColIndex = 0;
    layoutRowIndex = 0;

    Command stopAllCommand = DriverControls.getResetDisableAllSubsystemsCommand();
    stopAllCommand.setName("STOP ALL");
    commands.add(stopAllCommand);

    for (Command command : commands) {
      shooterCalibrationCommandLayout.add(command).withPosition(layoutColIndex, layoutRowIndex++);
    }

    return maxWidth;
  }

  public static void setupControls() {
    int colIndex = 0;
    int rowIndex = 0;
    ShuffleboardTab debugTab = Shuffleboard.getTab("Debug");

    setupshooterCalibrationControls(debugTab, colIndex, rowIndex, 2);
    colIndex += 2;

    GenericEntry intakeVoltageEntry =
        debugTab
            .add("Intake: Volts", IntakeConstants.defaultSpeedInVolts)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", 0, "max", 12))
            .withPosition(colIndex, rowIndex++)
            .withSize(2, 1)
            .getEntry();

    GenericEntry armAngleEntry =
        debugTab
            .add("Arm: Angle", ArmConstants.subwooferScoreAngleInDegrees)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(
                Map.of(
                    "min", ArmConstants.minAngleInDegrees, "max", ArmConstants.maxAngleInDegrees))
            .withPosition(colIndex, rowIndex++)
            .withSize(2, 1)
            .getEntry();

    GenericEntry shooterVelocityEntry =
        debugTab
            .add("Shooter: Velocity", ShooterConstants.velocityInRPM)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", 0, "max", 6000))
            .withPosition(colIndex, rowIndex++)
            .withSize(2, 1)
            .getEntry();

    debugTab
        .add(
            "Test: Shooter Angle",
            new TestShooterAngle(
                RobotConfig.shooter,
                RobotConfig.intake,
                RobotConfig.arm,
                () -> shooterVelocityEntry.getDouble(ShooterConstants.velocityInRPM),
                () -> intakeVoltageEntry.getDouble(IntakeConstants.defaultSpeedInVolts),
                () -> armAngleEntry.getDouble(ArmConstants.subwooferScoreAngleInDegrees)))
        .withPosition(colIndex, rowIndex++)
        .withSize(2, 1);

    colIndex += 2;
    rowIndex = 1;

    debugTab
        .add(
            "Arm To Position",
            new ArmToPosition(
                RobotConfig.arm,
                () -> armAngleEntry.getDouble(ArmConstants.subwooferScoreAngleInDegrees)))
        .withPosition(colIndex, rowIndex++)
        .withSize(2, 1);

    debugTab
        .add(
            "Shooter to Velocity",
            new SetShooterVelocity(
                RobotConfig.shooter,
                () -> shooterVelocityEntry.getDouble(ShooterConstants.velocityInRPM)))
        .withPosition(colIndex, rowIndex++)
        .withSize(2, 1);

    Command driveToAmp =
        AutoBuilder.pathfindToPoseFlipped(
            new Pose2d(1.8, 7.75, Rotation2d.fromDegrees(-90)),
            new PathConstraints(4.0, 3.0, 2 * Math.PI, 3 * Math.PI));
    driveToAmp.setName("Drive To Amp");
    debugTab.add(driveToAmp).withPosition(colIndex, rowIndex++).withSize(2, 1);
    ;
    /*
        colIndex += 2;
        rowIndex = 1;

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
    */
  }
}
