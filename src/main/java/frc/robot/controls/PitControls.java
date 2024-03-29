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
    maxWidth = Math.min(2, maxWidth);

    /* Intake Controls */
    ShuffleboardLayout intakeLayout =
        tab.getLayout("Intake", BuiltInLayouts.kGrid)
            .withProperties(Map.of("Label position", "TOP", "Number of columns", 1))
            .withSize(maxWidth, layoutMaxHeight)
            .withPosition(col, row);
    row += layoutMaxHeight;

    ShuffleboardLayout intakeStatusLayout =
        intakeLayout
            .getLayout("Status", BuiltInLayouts.kGrid)
            .withProperties(Map.of("Number of columns", 1))
            .withPosition(0, 0);
    layoutColIndex = 0;
    layoutRowIndex = 0;

    GenericEntry intakeVolts =
        intakeStatusLayout
            .add("Volts", IntakeConstants.defaultSpeedInVolts)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", -12, "max", 12))
            .withPosition(layoutColIndex, layoutRowIndex++)
            .getEntry();

    intakeStatusLayout
        .addDouble("Applied Volts", () -> RobotConfig.intake.getCurrentVoltage())
        .withWidget(BuiltInWidgets.kNumberBar)
        .withProperties(Map.of("min", -12, "max", 12))
        .withPosition(layoutColIndex, layoutRowIndex++);

    intakeStatusLayout
        .addBoolean("Piece Detected", () -> RobotConfig.intake.isPieceDetected())
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withProperties(Map.of("Color when false", "black", "Color when true", "orange"))
        .withPosition(layoutColIndex, layoutRowIndex++);

    ShuffleboardLayout intakeCommandLayout =
        intakeLayout
            .getLayout("Commands", BuiltInLayouts.kGrid)
            .withProperties(Map.of("Label position", "HIDDEN", "Number of columns", 1))
            .withPosition(0, 1);
    layoutColIndex = 0;
    layoutRowIndex = 0;

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
      intakeCommandLayout.add(command).withPosition(layoutColIndex, layoutRowIndex++);
    }

    return maxWidth;
  }

  private static int setupArmControls(ShuffleboardTab tab, int col, int row, int maxWidth) {
    int layoutColIndex = 0;
    int layoutRowIndex = 0;
    int layoutMaxHeight = 5;
    List<Command> commands = new ArrayList<Command>();
    maxWidth = Math.min(2, maxWidth);

    ShuffleboardLayout armLayout =
        tab.getLayout("Arm", BuiltInLayouts.kGrid)
            .withProperties(Map.of("Label position", "TOP", "Number of columns", 1))
            .withSize(maxWidth, layoutMaxHeight)
            .withPosition(col, row);
    row += layoutMaxHeight;

    ShuffleboardLayout armStatusLayout =
        armLayout
            .getLayout("Status", BuiltInLayouts.kGrid)
            .withProperties(Map.of("Number of columns", 1))
            .withPosition(0, 0);
    layoutColIndex = 0;
    layoutRowIndex = 0;

    GenericEntry armVolts =
        armStatusLayout
            .add("Volts", ArmConstants.defaultSpeedInVolts)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", -12, "max", 12))
            .withPosition(layoutColIndex, layoutRowIndex++)
            .getEntry();

    armStatusLayout
        .addDouble("Abs Angle (degrees)", () -> RobotConfig.arm.getAngle())
        .withWidget(BuiltInWidgets.kNumberBar)
        .withProperties(
            Map.of("min", ArmConstants.minAngleInDegrees, "max", ArmConstants.maxAngleInDegrees))
        //        .withSize(maxWidth, 1)
        .withPosition(layoutColIndex, layoutRowIndex++);

    armStatusLayout
        .addDouble("Rel Angle (degrees)", () -> RobotConfig.arm.getRelativeAngle())
        .withWidget(BuiltInWidgets.kNumberBar)
        .withProperties(
            Map.of("min", ArmConstants.minAngleInDegrees, "max", ArmConstants.maxAngleInDegrees))
        //        .withSize(maxWidth, 1)
        .withPosition(layoutColIndex, layoutRowIndex++);

    armStatusLayout
        .addBoolean("High Limit", () -> RobotConfig.arm.isAtMaxLimit())
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withProperties(Map.of("Color when false", "black", "Color when true", "red"))
        .withPosition(layoutColIndex, layoutRowIndex++);

    armStatusLayout
        .addBoolean("Low Limit", () -> RobotConfig.arm.isAtMinLimit())
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withProperties(Map.of("Color when false", "black", "Color when true", "red"))
        .withPosition(layoutColIndex, layoutRowIndex++);

    ShuffleboardLayout armCommandLayout =
        armLayout
            .getLayout("Commands", BuiltInLayouts.kGrid)
            .withProperties(Map.of("Label position", "HIDDEN", "Number of columns", 1))
            .withPosition(0, 1);
    layoutColIndex = 0;
    layoutRowIndex = 0;

    Command armStop = new InstantCommand(() -> RobotConfig.arm.runVoltage(0), RobotConfig.arm);
    armStop.setName("Stop");
    commands.add(armStop);

    Command armOn =
        new InstantCommand(
            () -> RobotConfig.arm.runVoltage(armVolts.getDouble(ArmConstants.defaultSpeedInVolts)),
            RobotConfig.arm);
    armOn.setName("On");
    commands.add(armOn);

    /*
    Command PrepareArmForMatch =
        new SequentialCommandGroup(
            RobotConfig.climber.getAutoZeroCommand(),
            new ArmToPosition(RobotConfig.arm, () -> ArmConstants.matchStartArmAngle),
            RobotConfig.climber.getPrepareClimberToHoldArmCommand(),
            new InstantCommand(() -> DevilBotState.climberNeedsToBeZeroedAtStart = true));
    PrepareArmForMatch.setName("Prepare Arm For Match");
    commands.add(PrepareArmForMatch);
    */

    for (Command command : commands) {
      armCommandLayout.add(command).withPosition(layoutColIndex, layoutRowIndex++);
    }

    return maxWidth;
  }

  private static int setupShooterControls(ShuffleboardTab tab, int col, int row, int maxWidth) {
    int layoutColIndex = 0;
    int layoutRowIndex = 0;
    int layoutMaxHeight = 3;
    List<Command> commands = new ArrayList<Command>();
    maxWidth = Math.min(2, maxWidth);

    ShuffleboardLayout shooterLayout =
        tab.getLayout("Shooter", BuiltInLayouts.kGrid)
            .withProperties(Map.of("Label position", "TOP", "Number of columns", 1))
            .withSize(maxWidth, layoutMaxHeight)
            .withPosition(col, row);
    row += layoutMaxHeight;

    ShuffleboardLayout shooterStatusLayout =
        shooterLayout
            .getLayout("Status", BuiltInLayouts.kGrid)
            .withProperties(Map.of("Number of columns", 1))
            .withPosition(0, 0);
    layoutColIndex = 0;
    layoutRowIndex = 0;

    // Create volt entry under Shooter tab as a number sider with min = -1 and max = 1
    GenericEntry shooterVolts =
        shooterStatusLayout
            .add("Volts", ShooterConstants.defaultSpeedInVolts)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", -12, "max", 12))
            .withPosition(layoutColIndex, layoutRowIndex++)
            .getEntry();
    shooterStatusLayout
        .addDouble(
            "Velocity (RPMs)",
            () -> Units.radiansPerSecondToRotationsPerMinute(RobotConfig.shooter.getCurrentSpeed()))
        .withWidget(BuiltInWidgets.kNumberBar)
        .withProperties(Map.of("min", 0, "max", ShooterConstants.maxVelocityInRPM))
        .withPosition(layoutColIndex, layoutRowIndex++);

    ShuffleboardLayout shooterCommandLayout =
        shooterLayout
            .getLayout("Commands", BuiltInLayouts.kGrid)
            .withProperties(Map.of("Label position", "HIDDEN", "Number of columns", 1))
            .withPosition(0, 1);
    layoutColIndex = 0;
    layoutRowIndex = 0;

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
      shooterCommandLayout.add(command).withPosition(layoutColIndex, layoutRowIndex++);
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
            .withProperties(Map.of("Label position", "TOP", "Number of columns", 1))
            .withSize(maxWidth, layoutMaxHeight)
            .withPosition(col, row);
    row += layoutMaxHeight;

    ShuffleboardLayout climberLowLevelStatusLayout =
        climberLowLevelLayout
            .getLayout("Status", BuiltInLayouts.kGrid)
            .withProperties(Map.of("Number of columns", 1))
            .withPosition(0, 0);
    layoutColIndex = 0;
    layoutRowIndex = 0;

    GenericEntry climberVolts =
        climberLowLevelStatusLayout
            .add("Volts", ClimberConstants.defaultSpeedInVolts)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", 0, "max", 12))
            .withPosition(layoutColIndex, layoutRowIndex++)
            .getEntry();

    ShuffleboardLayout climberLowLevelCommandLayout =
        climberLowLevelLayout
            .getLayout("Commands", BuiltInLayouts.kGrid)
            .withProperties(Map.of("Label position", "HIDDEN", "Number of columns", 2))
            .withPosition(0, 1);
    layoutColIndex = 0;
    layoutRowIndex = 0;

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
      climberLowLevelCommandLayout.add(command).withPosition(layoutColIndex++, layoutRowIndex);
    }

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
            .withProperties(Map.of("Label position", "TOP", "Number of columns", 1))
            .withSize(maxWidth, layoutMaxHeight)
            .withPosition(col, row);
    row += layoutMaxHeight;

    ShuffleboardLayout climberStatusLayout =
        climberLayout
            .getLayout("Status", BuiltInLayouts.kGrid)
            .withProperties(Map.of("Number of columns", 2))
            .withPosition(0, 0);
    layoutColIndex = 0;
    layoutRowIndex = 0;

    climberStatusLayout
        .addDouble("L Position", () -> RobotConfig.climber.getCurrentPositionLeft())
        .withWidget(BuiltInWidgets.kNumberBar)
        .withProperties(Map.of("min", 0, "max", ClimberConstants.maxPositionInRadians))
        .withPosition(layoutColIndex++, layoutRowIndex);

    climberStatusLayout
        .addDouble("R Position", () -> RobotConfig.climber.getCurrentPositionRight())
        .withWidget(BuiltInWidgets.kNumberBar)
        .withProperties(Map.of("min", 0, "max", ClimberConstants.maxPositionInRadians))
        .withPosition(layoutColIndex, layoutRowIndex++);
    layoutColIndex = 0;

    climberStatusLayout
        .addBoolean("L High Limit", () -> RobotConfig.climber.isAtMaxLimitLeft())
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withProperties(Map.of("Color when false", "black", "Color when true", "red"))
        .withPosition(layoutColIndex++, layoutRowIndex);

    climberStatusLayout
        .addBoolean("R High Limit", () -> RobotConfig.climber.isAtMaxLimitRight())
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withProperties(Map.of("Color when false", "black", "Color when true", "red"))
        .withPosition(layoutColIndex++, layoutRowIndex++);
    layoutColIndex = 0;

    climberStatusLayout
        .addBoolean("L Low Limit", () -> RobotConfig.climber.isAtMinLimitLeft())
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withProperties(Map.of("Color when false", "black", "Color when true", "red"))
        .withPosition(layoutColIndex++, layoutRowIndex);

    climberStatusLayout
        .addBoolean("R Low Limit", () -> RobotConfig.climber.isAtMinLimitRight())
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withProperties(Map.of("Color when false", "black", "Color when true", "red"))
        .withPosition(layoutColIndex, layoutRowIndex);
    layoutColIndex = 0;

    ShuffleboardLayout climberCommandLayout =
        climberLayout
            .getLayout("Commands", BuiltInLayouts.kGrid)
            .withProperties(Map.of("Label position", "HIDDEN", "Number of columns", 1))
            .withPosition(0, 1);
    layoutColIndex = 0;
    layoutRowIndex = 0;

    Command climberStop =
        new SequentialCommandGroup(
            new InstantCommand(() -> RobotConfig.climber.runVoltage(0), RobotConfig.climber),
            new InstantCommand(() -> RobotConfig.climber.enableLimits(true), RobotConfig.climber));
    climberStop.setName("Stop");
    commands.add(climberStop);

    Command climberAutoZero = RobotConfig.climber.getAutoZeroCommand();

    climberAutoZero.setName("Auto Zero");
    commands.add(climberAutoZero);

    Command climberExtend = RobotConfig.climber.getExtendCommand();
    climberExtend.setName("Extend");
    commands.add(climberExtend);

    Command climberRetract = RobotConfig.climber.getRetractCommand();
    climberRetract.setName("Retract");
    commands.add(climberRetract);

    for (Command command : commands) {
      climberCommandLayout
          .add(command)
          .withProperties(Map.of("Label position", "HIDDEN"))
          .withPosition(layoutColIndex, layoutRowIndex++);
    }

    setupLowLevelClimberControls(tab, col, row, maxWidth);

    return maxWidth;
  }

  private static int setupVisionControls(ShuffleboardTab tab, int col, int row, int maxWidth) {
    int layoutColIndex = 0;
    int layoutRowIndex = 0;
    int layoutMaxHeight = 2;
    //    List<Command> commands = new ArrayList<Command>();
    maxWidth = Math.min(2, maxWidth);

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
