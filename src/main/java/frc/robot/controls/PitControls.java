package frc.robot.controls;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.climber.ClimberCommand;
import frc.robot.config.RobotConfig;
import frc.robot.config.RobotConfig.ArmConstants;
import frc.robot.config.RobotConfig.ClimberConstants;
import frc.robot.config.RobotConfig.IntakeConstants;
import frc.robot.config.RobotConfig.ShooterConstants;
import java.util.Map;

public class PitControls {
  public static void setupControls() {
    int colIndex = 0;
    int rowIndex = 0;
    ShuffleboardTab commandTestTab = Shuffleboard.getTab("Pit");

    commandTestTab.add("Intake: Command", RobotConfig.intake).withPosition(colIndex, rowIndex++);
    GenericEntry intakeVolts =
        commandTestTab
            .add("Intake: Volts", IntakeConstants.defaultSpeedInVolts)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", 0, "max", 12))
            .withPosition(colIndex, rowIndex++)
            .withSize(2, 1)
            .getEntry();
    commandTestTab
        .add(
            "Intake: Stop",
            new InstantCommand(() -> RobotConfig.intake.runVoltage(0), RobotConfig.intake))
        .withPosition(colIndex, rowIndex++)
        .withSize(2, 1);
    commandTestTab
        .add(
            "Intake: In",
            new InstantCommand(
                () ->
                    RobotConfig.intake.runVoltage(
                        intakeVolts.getDouble(IntakeConstants.defaultSpeedInVolts)),
                RobotConfig.intake))
        .withPosition(colIndex, rowIndex++)
        .withSize(2, 1);
    commandTestTab
        .add(
            "Intake: Out",
            new InstantCommand(
                () ->
                    RobotConfig.intake.runVoltage(
                        -intakeVolts.getDouble(IntakeConstants.defaultSpeedInVolts)),
                RobotConfig.intake))
        .withPosition(colIndex, rowIndex++)
        .withSize(2, 1);

    colIndex += 2;
    rowIndex = 0;

    commandTestTab.add("Climber: Command", RobotConfig.climber).withPosition(colIndex, rowIndex++);

    // Create volt entry under Shooter tab as a number sider with min = -1 and max = 1
    GenericEntry climberVolts =
        commandTestTab
            .add("Climber: Volts", ClimberConstants.defaultSpeedInVolts)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", 0, "max", 12))
            .withPosition(colIndex, rowIndex++)
            .getEntry();
    commandTestTab
        .add(
            "Climber: Stop",
            new SequentialCommandGroup(
                new InstantCommand(() -> RobotConfig.climber.runVoltage(0), RobotConfig.climber),
                new InstantCommand(
                    () -> RobotConfig.climber.autoZeroMode(false), RobotConfig.climber),
                new InstantCommand(
                    () -> RobotConfig.climber.enableLimits(true), RobotConfig.climber)))
        .withPosition(colIndex, rowIndex)
        .withSize(2, 1);
    /*
            commandTestTab
            .add(
                "Climber: Zero",
                new InstantCommand(() -> RobotConfig.climber.resetPosition(), RobotConfig.climber))
            .withPosition(colIndex + 1, rowIndex);
    */

    rowIndex++;
    commandTestTab
        .add(
            "Climber: Auto Zero",
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
                    RobotConfig.climber)))
        .withPosition(colIndex, rowIndex)
        .withSize(2, 1);

    rowIndex++;
    commandTestTab
        .add("Climber: Extend", new ClimberCommand(RobotConfig.climber, true))
        .withPosition(colIndex, rowIndex++)
        .withSize(2, 1);
    commandTestTab
        .add("Climber: Retract", new ClimberCommand(RobotConfig.climber, false))
        .withPosition(colIndex, rowIndex++)
        .withSize(2, 1);

    rowIndex++;
    commandTestTab
        .add(
            "Climber: L Up (No Limit!)",
            new InstantCommand(
                () -> {
                  RobotConfig.climber.enableLimits(false);
                  RobotConfig.climber.runVoltageLeft(
                      climberVolts.getDouble(ClimberConstants.defaultSpeedInVolts));
                },
                RobotConfig.climber))
        .withPosition(colIndex + 0, rowIndex);

    commandTestTab
        .add(
            "Climber: R Up (No Limit!)",
            new InstantCommand(
                () -> {
                  RobotConfig.climber.enableLimits(false);

                  RobotConfig.climber.runVoltageRight(
                      climberVolts.getDouble(ClimberConstants.defaultSpeedInVolts));
                },
                RobotConfig.climber))
        .withPosition(colIndex + 1, rowIndex);

    rowIndex++;
    commandTestTab
        .add(
            "Climber: L Down (No Limit!)",
            new SequentialCommandGroup(
                new InstantCommand(() -> RobotConfig.climber.enableLimits(false)),
                new InstantCommand(
                    () ->
                        RobotConfig.climber.runVoltageLeft(
                            -climberVolts.getDouble(ClimberConstants.defaultSpeedInVolts)),
                    RobotConfig.climber)))
        .withPosition(colIndex + 0, rowIndex);
    commandTestTab
        .add(
            "Climber: R Down (No Limit!)",
            new SequentialCommandGroup(
                new InstantCommand(() -> RobotConfig.climber.enableLimits(false)),
                new InstantCommand(
                    () ->
                        RobotConfig.climber.runVoltageRight(
                            -climberVolts.getDouble(ClimberConstants.defaultSpeedInVolts)),
                    RobotConfig.climber)))
        .withPosition(colIndex + 1, rowIndex);

    colIndex += 2;
    rowIndex = 0;
    commandTestTab.add("Shooter: Command", RobotConfig.shooter).withPosition(colIndex, rowIndex++);
    // Create volt entry under Shooter tab as a number sider with min = -1 and max = 1
    GenericEntry shooterVolts =
        commandTestTab
            .add("Shooter: Volts", ShooterConstants.defaultSpeedInVolts)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", -12, "max", 12))
            .withPosition(colIndex, rowIndex++)
            .getEntry();
    commandTestTab
        .add(
            "Shooter: Stop",
            new InstantCommand(() -> RobotConfig.shooter.runVoltage(0), RobotConfig.shooter))
        .withPosition(colIndex, rowIndex++)
        .withSize(2, 1);
    commandTestTab
        .add(
            "Shooter: On",
            new InstantCommand(
                () ->
                    RobotConfig.shooter.runVoltage(
                        shooterVolts.getDouble(ShooterConstants.defaultSpeedInVolts)),
                RobotConfig.shooter))
        .withPosition(colIndex, rowIndex++)
        .withSize(2, 1);

    colIndex += 2;
    rowIndex = 0;
    commandTestTab.add("Arm: Command", RobotConfig.arm).withPosition(colIndex, rowIndex++);
    // Create volt entry under Shooter tab as a number sider with min = -1 and max = 1
    GenericEntry armVolts =
        commandTestTab
            .add("Arm: Volts", ArmConstants.defaultSpeedInVolts)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", -12, "max", 12))
            .withPosition(colIndex, rowIndex++)
            .getEntry();
    commandTestTab
        .add("Arm: Stop", new InstantCommand(() -> RobotConfig.arm.runVoltage(0), RobotConfig.arm))
        .withPosition(colIndex, rowIndex++)
        .withSize(2, 1);
    commandTestTab
        .add(
            "Arm: On",
            new InstantCommand(
                () ->
                    RobotConfig.arm.runVoltage(
                        armVolts.getDouble(ArmConstants.defaultSpeedInVolts)),
                RobotConfig.arm))
        .withPosition(colIndex, rowIndex++)
        .withSize(2, 1);

    commandTestTab.addDouble("Battery (volts)", ()-> RobotController.getBatteryVoltage())
    .withWidget(BuiltInWidgets.kVoltageView)
    .withPosition(colIndex, rowIndex++)
        .withSize(2, 1);      

    commandTestTab.addDouble("Shooter Velocity (RPMs)", ()-> Units.radiansPerSecondToRotationsPerMinute(RobotConfig.shooter.getCurrentSpeed()))
    .withWidget(BuiltInWidgets.kNumberBar)
    .withPosition(colIndex, rowIndex++)
    .withSize(2, 1);      

    commandTestTab.addDouble("Arm Angle (degrees)", ()-> RobotConfig.arm.getAngle())
    .withWidget(BuiltInWidgets.kNumberBar)
    .withPosition(colIndex, rowIndex++)
        .withSize(2, 1);

    commandTestTab.addDouble("Yaw (degrees)", ()-> RobotConfig.drive.getAngle() % 180)
    .withWidget(BuiltInWidgets.kDial)
    .withProperties(Map.of("min", -180, "max", 180))
    .withPosition(colIndex, rowIndex++)
        .withSize(2, 1);      


  }
}
