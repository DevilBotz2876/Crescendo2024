package frc.robot.controls;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.config.RobotConfig;
import frc.robot.config.RobotConfig.ArmConstants;
import frc.robot.config.RobotConfig.ClimberConstants;
import frc.robot.config.RobotConfig.IntakeConstants;
import frc.robot.config.RobotConfig.ShooterConstants;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Optional;

public class PitControls {
  private static int setupIntakeControls(ShuffleboardTab tab, int col, int row, int maxWidth) {
    int layoutColIndex = 0;
    int layoutRowIndex = 0;
    int layoutMaxHeight = 4;
    List<Command> commands = new ArrayList<Command>();
    maxWidth = Math.min(1, maxWidth);

    /* Intake Controls */
    ShuffleboardLayout intakeLayout =
        tab.getLayout("Intake", BuiltInLayouts.kGrid)
            //            .withProperties(Map.of("Label position", "HIDDEN"))
            .withSize(maxWidth, layoutMaxHeight)
            .withPosition(col, row);
    row += layoutMaxHeight;

    GenericEntry intakeVolts =
        intakeLayout
            .add("Volts", IntakeConstants.defaultSpeedInVolts)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", -12, "max", 12))
            .withPosition(layoutColIndex, layoutRowIndex++)
            //            .withSize(maxWidth, 1)
            .getEntry();

    intakeLayout
        .addDouble("Applied Volts", () -> RobotConfig.intake.getCurrentVoltage())
        .withWidget(BuiltInWidgets.kNumberBar)
        .withProperties(Map.of("min", -12, "max", 12))
        //        .withSize(maxWidth, 1)
        .withPosition(layoutColIndex, layoutRowIndex++);

    intakeLayout
        .addBoolean("Piece Detected", () -> RobotConfig.intake.isPieceDetected())
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withProperties(Map.of("Color when false", "black", "Color when true", "orange"))
        .withPosition(layoutColIndex, layoutRowIndex++);

    Command intakeStop =
        new InstantCommand(() -> RobotConfig.intake.runVoltage(0), RobotConfig.intake);
    intakeStop.setName(("Stop"));
    commands.add(intakeStop);

    Command intakeOn =
        new InstantCommand(
            () ->
                RobotConfig.intake.runVoltage(
                    intakeVolts.getDouble(IntakeConstants.defaultSpeedInVolts)),
            RobotConfig.intake);
    intakeOn.setName("On");
    commands.add(intakeOn);

    for (Command command : commands) {
      intakeLayout
          .add(command)
          //        .withSize(maxWidth, 1)
          .withPosition(layoutColIndex, layoutRowIndex++);
    }

    return maxWidth;
  }

  private static int setupArmControls(ShuffleboardTab tab, int col, int row, int maxWidth) {
    int layoutColIndex = 0;
    int layoutRowIndex = 0;
    int layoutMaxHeight = 5;
    List<Command> commands = new ArrayList<Command>();
    maxWidth = Math.min(1, maxWidth);

    ShuffleboardLayout armLayout =
        tab.getLayout("Arm", BuiltInLayouts.kGrid)
            //            .withProperties(Map.of("Label position", "HIDDEN"))
            .withSize(maxWidth, layoutMaxHeight)
            .withPosition(col, row);
    row += layoutMaxHeight;

    GenericEntry armVolts =
        armLayout
            .add("Volts", ArmConstants.defaultSpeedInVolts)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", -12, "max", 12))
            //            .withSize(maxWidth, 1)
            .withPosition(layoutColIndex, layoutRowIndex++)
            .getEntry();

    armLayout
        .addDouble("Abs Angle (degrees)", () -> RobotConfig.arm.getAngle())
        .withWidget(BuiltInWidgets.kNumberBar)
        .withProperties(
            Map.of("min", ArmConstants.minAngleInDegrees, "max", ArmConstants.maxAngleInDegrees))
        //        .withSize(maxWidth, 1)
        .withPosition(layoutColIndex, layoutRowIndex++);

    armLayout
        .addDouble("Rel Angle (degrees)", () -> RobotConfig.arm.getRelativeAngle())
        .withWidget(BuiltInWidgets.kNumberBar)
        .withProperties(
            Map.of("min", ArmConstants.minAngleInDegrees, "max", ArmConstants.maxAngleInDegrees))
        //        .withSize(maxWidth, 1)
        .withPosition(layoutColIndex, layoutRowIndex++);

    armLayout
        .addBoolean("High Limit", () -> RobotConfig.arm.isAtMaxLimit())
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withProperties(Map.of("Color when false", "black", "Color when true", "red"))
        .withPosition(layoutColIndex, layoutRowIndex++);

    armLayout
        .addBoolean("Low Limit", () -> RobotConfig.arm.isAtMinLimit())
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withProperties(Map.of("Color when false", "black", "Color when true", "red"))
        .withPosition(layoutColIndex, layoutRowIndex++);

    Command armStop = new InstantCommand(() -> RobotConfig.arm.runVoltage(0), RobotConfig.arm);
    armStop.setName("Stop");
    commands.add(armStop);

    Command armOn =
        new InstantCommand(
            () -> RobotConfig.arm.runVoltage(armVolts.getDouble(ArmConstants.defaultSpeedInVolts)),
            RobotConfig.arm);
    armOn.setName("On");
    commands.add(armOn);

    for (Command command : commands) {
      armLayout
          .add(command)
          //        .withSize(maxWidth, 1)
          .withPosition(layoutColIndex, layoutRowIndex++);
    }

    return maxWidth;
  }

  private static int setupShooterControls(ShuffleboardTab tab, int col, int row, int maxWidth) {
    int layoutColIndex = 0;
    int layoutRowIndex = 0;
    int layoutMaxHeight = 3;
    List<Command> commands = new ArrayList<Command>();
    maxWidth = Math.min(1, maxWidth);

    ShuffleboardLayout shooterLayout =
        tab.getLayout("Shooter", BuiltInLayouts.kGrid)
            //        .withProperties(Map.of("Label position", "HIDDEN"))
            .withSize(maxWidth, layoutMaxHeight)
            .withPosition(col, row);
    row += layoutMaxHeight;

    // Create volt entry under Shooter tab as a number sider with min = -1 and max = 1
    GenericEntry shooterVolts =
        shooterLayout
            .add("Volts", ShooterConstants.defaultSpeedInVolts)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", -12, "max", 12))
            .withPosition(layoutColIndex, layoutRowIndex++)
            .getEntry();
    shooterLayout
        .addDouble(
            "Velocity (RPMs)",
            () -> Units.radiansPerSecondToRotationsPerMinute(RobotConfig.shooter.getCurrentSpeed()))
        .withWidget(BuiltInWidgets.kNumberBar)
        .withProperties(Map.of("min", 0, "max", ShooterConstants.maxVelocityInRPMs))
        //        .withSize(maxWidth, 1)
        .withPosition(layoutColIndex, layoutRowIndex++);

    Command shooterStop =
        new InstantCommand(() -> RobotConfig.shooter.runVoltage(0), RobotConfig.shooter);
    shooterStop.setName("Stop");
    commands.add(shooterStop);

    Command shooterOn =
        new InstantCommand(
            () ->
                RobotConfig.shooter.runVoltage(
                    shooterVolts.getDouble(ShooterConstants.defaultSpeedInVolts)),
            RobotConfig.shooter);
    shooterOn.setName("On");
    commands.add(shooterOn);

    for (Command command : commands) {
      shooterLayout
          .add(command)
          //        .withSize(maxWidth, 1)
          .withPosition(layoutColIndex, layoutRowIndex++);
    }

    return maxWidth;
  }

  private static int setupLowLevelClimberControls(
      ShuffleboardTab tab, int col, int row, int maxWidth) {
    int layoutColIndex = 0;
    int layoutRowIndex = 0;
    int layoutMaxHeight = 2;
    List<Command> commands = new ArrayList<Command>();
    maxWidth = Math.min(2, maxWidth);

    ShuffleboardLayout climberLowLevelLayout =
        tab.getLayout("Climber (Low Level)", BuiltInLayouts.kGrid)
            //            .withProperties(Map.of("Label position", "HIDDEN"))
            .withSize(maxWidth, layoutMaxHeight)
            .withPosition(col, row);
    row += layoutMaxHeight;

    GenericEntry climberVolts =
        climberLowLevelLayout
            .add("Volts", ClimberConstants.defaultSpeedInVolts)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", 0, "max", 12))
            .withPosition(layoutColIndex, layoutRowIndex++)
            //            .withSize(maxWidth, 1)
            .getEntry();

    Command climberLeftUp =
        new InstantCommand(
            () -> {
              RobotConfig.climber.enableLimits(false);
              RobotConfig.climber.runVoltageLeft(
                  climberVolts.getDouble(ClimberConstants.defaultSpeedInVolts));
            },
            RobotConfig.climber);
    climberLeftUp.setName("L Up");
    commands.add(climberLeftUp);

    Command climberRightUp =
        new InstantCommand(
            () -> {
              RobotConfig.climber.enableLimits(false);
              RobotConfig.climber.runVoltageRight(
                  climberVolts.getDouble(ClimberConstants.defaultSpeedInVolts));
            },
            RobotConfig.climber);
    climberRightUp.setName("R Up");
    commands.add(climberRightUp);

    Command climberLeftDown =
        new InstantCommand(
            () -> {
              RobotConfig.climber.enableLimits(false);
              RobotConfig.climber.runVoltageLeft(
                  -climberVolts.getDouble(ClimberConstants.defaultSpeedInVolts));
            },
            RobotConfig.climber);
    climberLeftDown.setName("L Down");
    commands.add(climberLeftDown);

    Command climberRightDown =
        new InstantCommand(
            () -> {
              RobotConfig.climber.enableLimits(false);
              RobotConfig.climber.runVoltageRight(
                  -climberVolts.getDouble(ClimberConstants.defaultSpeedInVolts));
            },
            RobotConfig.climber);
    climberRightDown.setName("R Down");
    commands.add(climberRightDown);

    for (Command command : commands) {
      if (layoutColIndex >= 2) {
        layoutColIndex = 0;
        layoutRowIndex++;
      }
      climberLowLevelLayout
          .add(command)
          .withSize(maxWidth, 1)
          .withPosition(layoutColIndex++, layoutRowIndex);
    }

    /*
            commandTestTab
            .add(
                "Zero",
                new InstantCommand(() -> RobotConfig.climber.resetPosition(), RobotConfig.climber))
            .withPosition(colIndex + 1, rowIndex);
    */

    return maxWidth;
  }

  private static int setupClimberControls(ShuffleboardTab tab, int col, int row, int maxWidth) {
    int layoutColIndex = 0;
    int layoutRowIndex = 0;
    int layoutMaxHeight = 4;
    List<Command> commands = new ArrayList<Command>();
    maxWidth = Math.min(2, maxWidth);

    /* Climber Controls */
    ShuffleboardLayout climberLayout =
        tab.getLayout("Climber", BuiltInLayouts.kGrid)
            //            .withProperties(Map.of("Label position", "HIDDEN"))
            .withSize(maxWidth, layoutMaxHeight)
            .withPosition(col, row);
    row += layoutMaxHeight;

    climberLayout
        .addDouble("L Position", () -> RobotConfig.climber.getCurrentPositionLeft())
        .withWidget(BuiltInWidgets.kNumberBar)
        .withProperties(Map.of("min", 0, "max", ClimberConstants.maxPositionInRadians))
        //        .withSize(maxWidth, 1)
        .withPosition(layoutColIndex++, layoutRowIndex);

    climberLayout
        .addDouble("R Position", () -> RobotConfig.climber.getCurrentPositionRight())
        .withWidget(BuiltInWidgets.kNumberBar)
        .withProperties(Map.of("min", 0, "max", ClimberConstants.maxPositionInRadians))
        //            .withSize(maxWidth, 1)
        .withPosition(layoutColIndex, layoutRowIndex++);
    layoutColIndex = 0;

    climberLayout
        .addBoolean("L High Limit", () -> RobotConfig.climber.isAtMaxLimitLeft())
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withProperties(Map.of("Color when false", "black", "Color when true", "red"))
        .withPosition(layoutColIndex++, layoutRowIndex);

    climberLayout
        .addBoolean("R High Limit", () -> RobotConfig.climber.isAtMaxLimitRight())
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withProperties(Map.of("Color when false", "black", "Color when true", "red"))
        .withPosition(layoutColIndex++, layoutRowIndex);

    layoutColIndex = 0;
    layoutRowIndex++;

    climberLayout
        .addBoolean("L Low Limit", () -> RobotConfig.climber.isAtMinLimitLeft())
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withProperties(Map.of("Color when false", "black", "Color when true", "red"))
        .withPosition(layoutColIndex++, layoutRowIndex);

    climberLayout
        .addBoolean("R Low Limit", () -> RobotConfig.climber.isAtMinLimitRight())
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withProperties(Map.of("Color when false", "black", "Color when true", "red"))
        .withPosition(layoutColIndex, layoutRowIndex);

    layoutColIndex = 0;
    layoutRowIndex++;

    Command climberStop =
        new SequentialCommandGroup(
            new InstantCommand(() -> RobotConfig.climber.runVoltage(0), RobotConfig.climber),
            new InstantCommand(() -> RobotConfig.climber.autoZeroMode(false), RobotConfig.climber),
            new InstantCommand(() -> RobotConfig.climber.enableLimits(true), RobotConfig.climber));
    climberStop.setName("Stop");
    commands.add(climberStop);

    Command climberAutoZero =
        new SequentialCommandGroup(
            new InstantCommand(
                () -> {
                  RobotConfig.climber.enableLimits(false);
                  RobotConfig.climber.runVoltage(ClimberConstants.autoZeroVoltage);
                },
                RobotConfig.climber),
            new WaitCommand(ClimberConstants.autoZeroExtendTimeInSeconds),
            new InstantCommand(
                () -> {
                  RobotConfig.climber.enableLimits(true);
                  RobotConfig.climber.autoZeroMode(true);
                  RobotConfig.climber.runVoltage(-ClimberConstants.autoZeroVoltage);
                },
                RobotConfig.climber),
            new WaitCommand(ClimberConstants.autoZeroMaxRetractTimeInSeconds),
            new InstantCommand(
                () -> {
                  RobotConfig.climber.autoZeroMode(false);
                  RobotConfig.climber.runVoltage(0);
                },
                RobotConfig.climber));
    climberAutoZero.setName("Zero");
    commands.add(climberAutoZero);

    Command climberExtend = RobotConfig.climber.getExtendCommand();
    climberExtend.setName("Extend");
    commands.add(climberExtend);

    Command climberRetract = RobotConfig.climber.getRetractCommand();
    climberRetract.setName("Retract");
    commands.add(climberRetract);

    for (Command command : commands) {
      if (layoutColIndex >= 2) {
        layoutColIndex = 0;
        layoutRowIndex++;
      }

      climberLayout
          .add(command)
          //        .withSize(maxWidth, 1)
          .withPosition(layoutColIndex++, layoutRowIndex);
    }

    setupLowLevelClimberControls(tab, col, row, maxWidth);

    return maxWidth;
  }

  private static int setupVisionControls(ShuffleboardTab tab, int col, int row, int maxWidth) {
    int layoutColIndex = 0;
    int layoutRowIndex = 0;
    int layoutMaxHeight = 2;
    //    List<Command> commands = new ArrayList<Command>();
    maxWidth = Math.min(1, maxWidth);

    /* Vision Controls */
    ShuffleboardLayout visionLayout =
        tab.getLayout("Vision", BuiltInLayouts.kGrid)
            //            .withProperties(Map.of("Label position", "HIDDEN"))
            .withSize(maxWidth, layoutMaxHeight)
            .withPosition(col, row);
    row += layoutMaxHeight;

    visionLayout
        .addDouble(
            "April Tag ID",
            () -> {
              Optional<Integer> aprilTagId = RobotConfig.vision.getBestTargetId();
              if (aprilTagId.isPresent()) return aprilTagId.get();
              else return -1;
            })
        .withWidget(BuiltInWidgets.kTextView)
        .withPosition(layoutColIndex, layoutRowIndex++);

    visionLayout
        .addDouble(
            "Yaw",
            () -> {
              Optional<Double> yaw = RobotConfig.vision.getYawToBestTarget();
              if (yaw.isPresent()) return yaw.get();
              else return 0;
            })
        .withWidget(BuiltInWidgets.kTextView)
        .withPosition(layoutColIndex, layoutRowIndex++);

    visionLayout
        .addDouble(
            "Distance",
            () -> {
              Optional<Double> distance = RobotConfig.vision.getDistanceToBestTarget();
              if (distance.isPresent()) return distance.get();
              else return -1;
            })
        .withWidget(BuiltInWidgets.kTextView)
        .withPosition(layoutColIndex, layoutRowIndex++);

    return maxWidth;
  }

  public static void setupControls() {
    int maxWidth = 3;
    int colIndex = 0;
    int rowIndex = 0;
    ShuffleboardTab pitTab = Shuffleboard.getTab("Pit");

    colIndex += setupIntakeControls(pitTab, colIndex, rowIndex, maxWidth);
    colIndex += setupArmControls(pitTab, colIndex, rowIndex, maxWidth);
    colIndex += setupShooterControls(pitTab, colIndex, rowIndex, maxWidth);
    colIndex += setupClimberControls(pitTab, colIndex, rowIndex, maxWidth);
    colIndex += setupVisionControls(pitTab, colIndex, rowIndex, maxWidth);

    colIndex = 0;
    rowIndex += 4;
    pitTab
        .addDouble("Battery (volts)", () -> RobotController.getBatteryVoltage())
        .withWidget(BuiltInWidgets.kVoltageView)
        .withProperties(Map.of("min", 0, "max", 12))
        .withSize(1, 1)
        .withPosition(colIndex, rowIndex++);

    pitTab
        .addDouble("Yaw (degrees)", () -> RobotConfig.drive.getAngle() % 180)
        .withWidget(BuiltInWidgets.kGyro)
        .withProperties(Map.of("min", -180, "max", 180))
        //        .withSize(maxWidth, 1)
        .withPosition(colIndex, rowIndex++);
  }
}
