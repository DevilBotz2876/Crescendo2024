package frc.robot.controls;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
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
  public static void setupGUI() {
    int colIndex = 0;
    int rowIndex = 0;
    // SmartDashboard.putData(new ArmToPosition(arm));
    ShuffleboardTab commandTestTab = Shuffleboard.getTab("Pit");
    NetworkTable commandGUI = NetworkTableInstance.getDefault().getTable("Shuffleboard/Pit");

    commandTestTab.add("Intake: Command", RobotConfig.intake).withPosition(colIndex, rowIndex++);
    commandTestTab
        .add("Intake: Volts", IntakeConstants.defaultSpeedInVolts)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", 0, "max", 12))
        .withPosition(colIndex, rowIndex++);
    commandTestTab
        .add(
            "Intake: Stop",
            new InstantCommand(() -> RobotConfig.intake.runVoltage(0), RobotConfig.intake))
        .withPosition(colIndex, rowIndex++);
    commandTestTab
        .add(
            "Intake: In",
            new InstantCommand(
                () ->
                    RobotConfig.intake.runVoltage(
                        commandGUI
                            .getEntry("Intake: Volts")
                            .getDouble(IntakeConstants.defaultSpeedInVolts)),
                RobotConfig.intake))
        .withPosition(colIndex, rowIndex++);
    commandTestTab
        .add(
            "Intake: Out",
            new InstantCommand(
                () ->
                    RobotConfig.intake.runVoltage(
                        -commandGUI
                            .getEntry("Intake: Volts")
                            .getDouble(IntakeConstants.defaultSpeedInVolts)),
                RobotConfig.intake))
        .withPosition(colIndex, rowIndex++);

    colIndex += 2;
    rowIndex = 0;

    commandTestTab.add("Climber: Command", RobotConfig.climber).withPosition(colIndex, rowIndex++);

    // Create volt entry under Shooter tab as a number sider with min = -1 and max = 1
    commandTestTab
        .add("Climber: Volts", ClimberConstants.defaultSpeedInVolts)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", 0, "max", 12))
        .withPosition(colIndex, rowIndex++);
    commandTestTab
        .add(
            "Climber: Stop",
            new SequentialCommandGroup(
                new InstantCommand(() -> RobotConfig.climber.runVoltage(0), RobotConfig.climber),
                new InstantCommand(
                    () -> RobotConfig.climber.autoZeroMode(false), RobotConfig.climber),
                new InstantCommand(
                    () -> RobotConfig.climber.enableLimits(true), RobotConfig.climber)))
        .withPosition(colIndex + 0, rowIndex);
    commandTestTab
        .add(
            "Climber: Zero",
            new InstantCommand(() -> RobotConfig.climber.resetPosition(), RobotConfig.climber))
        .withPosition(colIndex + 1, rowIndex);

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
        .add(
            "Climber: L Up (No Limit!)",
            new InstantCommand(
                () -> {
                  RobotConfig.climber.enableLimits(false);
                  RobotConfig.climber.runVoltageLeft(
                      commandGUI
                          .getEntry("Climber: Volts")
                          .getDouble(ClimberConstants.defaultSpeedInVolts));
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
                      commandGUI
                          .getEntry("Climber: Volts")
                          .getDouble(ClimberConstants.defaultSpeedInVolts));
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
                            -commandGUI
                                .getEntry("Climber: Volts")
                                .getDouble(ClimberConstants.defaultSpeedInVolts)),
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
                            -commandGUI
                                .getEntry("Climber: Volts")
                                .getDouble(ClimberConstants.defaultSpeedInVolts)),
                    RobotConfig.climber)))
        .withPosition(colIndex + 1, rowIndex);

    rowIndex++;
    commandTestTab
        .add("Climber: Extend", new ClimberCommand(RobotConfig.climber, true))
        .withPosition(colIndex, rowIndex++);
    commandTestTab
        .add("Climber: Retract", new ClimberCommand(RobotConfig.climber, false))
        .withPosition(colIndex, rowIndex++);

    colIndex += 2;
    rowIndex = 0;
    commandTestTab.add("Shooter: Command", RobotConfig.shooter).withPosition(colIndex, rowIndex++);
    // Create volt entry under Shooter tab as a number sider with min = -1 and max = 1
    commandTestTab
        .add("Shooter: Volts", ShooterConstants.defaultSpeedInVolts)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", -12, "max", 12))
        .withPosition(colIndex, rowIndex++);
    commandTestTab
        .add(
            "Shooter: Stop",
            new InstantCommand(() -> RobotConfig.shooter.runVoltage(0), RobotConfig.shooter))
        .withPosition(colIndex, rowIndex++);
    commandTestTab
        .add(
            "Shooter: On",
            new InstantCommand(
                () ->
                    RobotConfig.shooter.runVoltage(
                        commandGUI
                            .getEntry("Shooter: Volts")
                            .getDouble(ShooterConstants.defaultSpeedInVolts)),
                RobotConfig.shooter))
        .withPosition(colIndex, rowIndex++);

    colIndex += 2;
    rowIndex = 0;
    commandTestTab.add("Arm: Command", RobotConfig.arm).withPosition(colIndex, rowIndex++);
    // Create volt entry under Shooter tab as a number sider with min = -1 and max = 1
    commandTestTab
        .add("Arm: Volts", ArmConstants.defaultSpeedInVolts)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", -12, "max", 12))
        .withPosition(colIndex, rowIndex++);
    commandTestTab
        .add("Arm: Stop", new InstantCommand(() -> RobotConfig.arm.runVoltage(0), RobotConfig.arm))
        .withPosition(colIndex, rowIndex++);
    commandTestTab
        .add(
            "Arm: On",
            new InstantCommand(
                () ->
                    RobotConfig.arm.runVoltage(
                        commandGUI
                            .getEntry("Arm: Volts")
                            .getDouble(ArmConstants.defaultSpeedInVolts)),
                RobotConfig.arm))
        .withPosition(colIndex, rowIndex++);
  }
}
