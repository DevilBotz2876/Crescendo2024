// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.arm.ArmToPositionTP;
import frc.robot.commands.assist.EjectPiece;
import frc.robot.commands.assist.PrepareForIntake;
import frc.robot.commands.assist.PrepareForScore;
import frc.robot.commands.assist.ScorePiece;
import frc.robot.commands.climber.ClimberCommand;
import frc.robot.commands.drive.DriveCommand;
import frc.robot.commands.shooter.TestShooterAngle;
import frc.robot.config.RobotConfig;
import frc.robot.config.RobotConfig.ArmConstants;
import frc.robot.config.RobotConfig.ClimberConstants;
import frc.robot.config.RobotConfig.IntakeConstants;
import frc.robot.config.RobotConfig.ShooterConstants;
import frc.robot.config.RobotConfigInferno;
import frc.robot.config.RobotConfigPhoenix;
import frc.robot.config.RobotConfigSherman;
import frc.robot.config.RobotConfigStub;
import java.util.Map;

public class RobotContainer {
  public final CommandXboxController controller;
  public final RobotConfig robotConfig;
  private static final String robotNameKey = "Robot Name";
  private static GenericEntry ampModeEntry = null;
  private static boolean ampMode = true;
  private static boolean speakerMode = true;
  private static GenericEntry fieldOrientedEntry = null;

  public RobotContainer() {
    String robotName = "UNKNOWN";
    controller = new CommandXboxController(0);

    Preferences.initString(robotNameKey, robotName);
    robotName = Preferences.getString(robotNameKey, robotName);
    System.out.println("Loading Settings for Robot Name = " + robotName);
    switch (robotName) {
      case "PHOENIX":
        robotConfig = new RobotConfigPhoenix();
        break;
      case "SHERMAN":
        robotConfig = new RobotConfigSherman();
        break;
      case "INFERNO":
        robotConfig = new RobotConfigInferno();
        break;
      case "UNKNOWN":
      default:
        /* If running simulation, put the robot config you want here */
        // robotConfig = new RobotConfigInferno();
        // robotConfig = new RobotConfigSherman();
        robotConfig = new RobotConfigStub();
    }

    // SmartDashboard.putData("Subsystems/Arm", RobotConfig.arm);
    // SmartDashboard.putData(CommandScheduler.getInstance());

    ShuffleboardTab autoTab = Shuffleboard.getTab("Autonomous");
    // Create volt entry under Shooter tab as a number sider with min = -1 and max = 1
    autoTab
        .add("Auto Chooser", RobotConfig.autoChooser)
        .withWidget(BuiltInWidgets.kComboBoxChooser);

    configureBindings();
    assistToShuffleboard();
    commandsToShuffleboard();
    sysIdToShuffleboard();
  }

  private void sysIdToShuffleboard() {
    int rowIndex = 0;
    int colIndex = 0;

    ShuffleboardTab sysIdTestTab = Shuffleboard.getTab("SysId");

    sysIdTestTab
        .add(
            "Arm: Quasistatic Forward",
            RobotConfig.arm.sysIdQuasistatic(SysIdRoutine.Direction.kForward))
        .withPosition(colIndex, rowIndex++)
        .withSize(2, 1);
    sysIdTestTab
        .add(
            "Arm: Quasistatic Reverse",
            RobotConfig.arm.sysIdQuasistatic(SysIdRoutine.Direction.kReverse))
        .withPosition(colIndex, rowIndex++)
        .withSize(2, 1);
    sysIdTestTab
        .add("Arm: Dynamic Forward", RobotConfig.arm.sysIdDynamic(SysIdRoutine.Direction.kForward))
        .withPosition(colIndex, rowIndex++)
        .withSize(2, 1);
    sysIdTestTab
        .add("Arm: Dynamic Reverse", RobotConfig.arm.sysIdDynamic(SysIdRoutine.Direction.kReverse))
        .withPosition(colIndex, rowIndex++)
        .withSize(2, 1);

    rowIndex = 0;
    colIndex += 2;
    sysIdTestTab
        .add(
            "Shooter: Quasistatic Forward",
            RobotConfig.shooter.sysIdQuasistatic(SysIdRoutine.Direction.kForward))
        .withPosition(colIndex, rowIndex++)
        .withSize(2, 1);
    sysIdTestTab
        .add(
            "Shooter: Quasistatic Reverse",
            RobotConfig.shooter.sysIdQuasistatic(SysIdRoutine.Direction.kReverse))
        .withPosition(colIndex, rowIndex++)
        .withSize(2, 1);
    sysIdTestTab
        .add(
            "Shooter: Dynamic Forward",
            RobotConfig.shooter.sysIdDynamic(SysIdRoutine.Direction.kForward))
        .withPosition(colIndex, rowIndex++)
        .withSize(2, 1);
    sysIdTestTab
        .add(
            "Shooter: Dynamic Reverse",
            RobotConfig.shooter.sysIdDynamic(SysIdRoutine.Direction.kReverse))
        .withPosition(colIndex, rowIndex++)
        .withSize(2, 1);

    /*
    colIndex++;
    rowIndex = 0;
    sysIdTestTab.add("Drive: Drive Motors", RobotConfig.drive.sysIdDriveMotorCommand()).withPosition(colIndex, rowIndex++).withSize(2, 1);
    sysIdTestTab.add("Drive: Angle Motors", RobotConfig.drive.sysIdAngleMotorCommand()).withPosition(colIndex, rowIndex++).withSize(2, 1);
    */
  }

  private void configureBindings() {
    // shooter.setDefaultCommand(new InstantCommand(() -> shooter.disable(), shooter));

    // Activate shooter to score piece in speaker or amp
    controller
        .rightTrigger()
        .onTrue(new ScorePiece(RobotConfig.intake, RobotConfig.shooter).withTimeout(1));

    // Move arm to some "safe" angle to protect intake while driving.
    controller
        .leftTrigger()
        .onTrue(
            new ArmToPositionTP(
                RobotConfig.ArmConstants.stowIntakeAngleInDegrees, RobotConfig.arm));

    // Try to spit piece out of intake in case it gets stuck
    controller
        .leftBumper()
        .onTrue(new EjectPiece(RobotConfig.intake, RobotConfig.arm).withTimeout(1));

    controller.a().onTrue(new PrepareForIntake(RobotConfig.arm, RobotConfig.intake));
    controller.b().onTrue(new PrepareForScore(RobotConfig.arm, RobotConfig.shooter, () -> ampMode));
    controller
        .y()
        .onTrue(new PrepareForScore(RobotConfig.arm, RobotConfig.shooter, () -> !ampMode));

    // Test Trapezoid Profile based arm movement
    // controller.y().onTrue(new ArmToPositionTP(75, RobotConfig.arm));
    // controller.x().onTrue(new ArmToPositionTP(45, RobotConfig.arm));
    // controller.a().onTrue(new ArmToPositionTP(0, RobotConfig.arm));

    // // Test PID arm movement
    // controller.y().onTrue(new ArmToPosition( RobotConfig.arm, () -> 75.0));
    // controller.x().onTrue(new ArmToPosition( RobotConfig.arm, () -> 45.0));
    // controller.a().onTrue(new ArmToPosition( RobotConfig.arm, () -> 0.0));

    // controller
    //     .y()
    //     .onTrue(
    //         new InstantCommand(
    //             () -> {
    //               ampMode = !ampMode;
    //               if (ampModeEntry != null) {
    //                 ampModeEntry.setBoolean(ampMode);
    //               }
    //             }));

    //    RobotConfig.intake.setDefaultCommand(
    //        new IntakeBaseCommand(
    //            RobotConfig.intake,
    //            () -> controller.rightBumper().getAsBoolean(),
    //            () -> controller.leftBumper().getAsBoolean()));

    RobotConfig.drive.setDefaultCommand(
        new DriveCommand(
            RobotConfig.drive,
            () -> MathUtil.applyDeadband(-controller.getLeftY(), 0.05),
            () -> MathUtil.applyDeadband(-controller.getLeftX(), 0.05),
            () -> MathUtil.applyDeadband(-controller.getRightX(), 0.05)));

    controller
        .x()
        .onTrue(
            new InstantCommand(
                () -> {
                  RobotConfig.drive.setFieldOrientedDrive(
                      !RobotConfig.drive.isFieldOrientedDrive());
                  if (fieldOrientedEntry != null) {
                    fieldOrientedEntry.setBoolean(RobotConfig.drive.isFieldOrientedDrive());
                  }
                },
                RobotConfig.drive));

    controller
        .back()
        .onTrue(new InstantCommand(() -> RobotConfig.drive.resetOdometry(), RobotConfig.drive));

    controller
        .start()
        .onTrue(
            new InstantCommand(
                () -> {
                  if (RobotConfig.climber.isExtending()) {
                    RobotConfig.climber.retract();
                  } else {
                    RobotConfig.climber.extend();
                  }
                },
                RobotConfig.climber));
  }

  public Command getAutonomousCommand() {
    return RobotConfig.autoChooser.getSelected();
  }

  public void commandsToShuffleboard() {
    int colIndex = 0;
    int rowIndex = 0;
    // SmartDashboard.putData(new ArmToPosition(arm));
    ShuffleboardTab commandTestTab = Shuffleboard.getTab("Commands");
    NetworkTable commandGUI = NetworkTableInstance.getDefault().getTable("Shuffleboard/Commands");

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

  void assistToShuffleboard() {
    int colIndex = 0;
    int rowIndex = 0;
    ShuffleboardTab assistTab = Shuffleboard.getTab("Assist");

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
            new PrepareForScore(RobotConfig.arm, RobotConfig.shooter, () -> ampMode))
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
        .add("Shooter: Angle (Amp)", ArmConstants.ampScoreShooterAngleInDegrees)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(
            Map.of("min", ArmConstants.minAngleInDegrees, "max", ArmConstants.maxAngleInDegrees))
        .withPosition(colIndex, rowIndex++)
        .withSize(2, 1);

    ampModeEntry =
        assistTab
            .add("Amp Mode", ampMode)
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

    fieldOrientedEntry =
        assistTab
            .add("Field Oriented Drive", RobotConfig.drive.isFieldOrientedDrive())
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withPosition(colIndex, rowIndex++)
            .withSize(2, 1)
            .getEntry();
  }
}
