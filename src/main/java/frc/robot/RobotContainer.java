// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.ArmToPositionDebug;
import frc.robot.commands.IntakeBaseCommand;
import frc.robot.commands.ShooterEnable;
import frc.robot.commands.drive.DriveCommand;
import frc.robot.config.RobotConfig;
import frc.robot.config.RobotConfigInferno;
import frc.robot.config.RobotConfigPhoenix;
import frc.robot.config.RobotConfigSherman;
import frc.robot.subsystems.arm.ArmIOSparkMax;
import frc.robot.subsystems.arm.ArmIOStub;
import frc.robot.subsystems.arm.ArmSubsystem;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

public class RobotContainer {
  public final CommandXboxController controller;
  public final RobotConfig robotConfig;
  public final ArmSubsystem arm;
  private SendableChooser<Command> autoChooser = null;
  private static final String robotNameKey = "Robot Name";

  private final LoggedDashboardNumber shooterSpeedInput =
      new LoggedDashboardNumber("Shooter Speed", 1000.0);

  public RobotContainer() {
    String robotName = "UNKNOWN";

    controller = new CommandXboxController(0);

    boolean hasArm = false;

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
        robotConfig = new RobotConfigInferno();
    }

    if (hasArm) {
      arm = new ArmSubsystem(new ArmIOSparkMax());
    } else {
      arm = new ArmSubsystem(new ArmIOStub());
    }

    configureBindings();
    // ArmSysIdBindings();
    // shooterSysIdBindings();
    // driveSysIdBindings();
  }

  private void ArmSysIdBindings() {
    controller.a().whileTrue(arm.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    controller.b().whileTrue(arm.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    controller.x().whileTrue(arm.sysIdDynamic(SysIdRoutine.Direction.kForward));
    controller.y().whileTrue(arm.sysIdDynamic(SysIdRoutine.Direction.kReverse));
  }

  private void shooterSysIdBindings() {
    controller.a().whileTrue(RobotConfig.shooter.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    controller.b().whileTrue(RobotConfig.shooter.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    controller.x().whileTrue(RobotConfig.shooter.sysIdDynamic(SysIdRoutine.Direction.kForward));
    controller.y().whileTrue(RobotConfig.shooter.sysIdDynamic(SysIdRoutine.Direction.kReverse));
  }

  private void driveSysIdBindings() {
    controller.a().whileTrue(RobotConfig.drive.sysIdDriveMotorCommand());
    controller.b().whileTrue(RobotConfig.drive.sysIdAngleMotorCommand());
  }

  private void configureBindings() {
    // shooter.setDefaultCommand(new InstantCommand(() -> shooter.disable(), shooter));
    controller.rightTrigger().whileTrue(new ShooterEnable(RobotConfig.shooter));

    controller
        .a()
        .whileTrue(
            Commands.startEnd(
                () -> RobotConfig.shooter.runVelocity(shooterSpeedInput.get()),
                () -> RobotConfig.shooter.runVelocity(0),
                RobotConfig.shooter));

    RobotConfig.intake.setDefaultCommand(
        new IntakeBaseCommand(
            RobotConfig.intake,
            () -> controller.rightBumper().getAsBoolean(),
            () -> controller.leftBumper().getAsBoolean()));

    RobotConfig.drive.setDefaultCommand(
        new DriveCommand(
            RobotConfig.drive,
            () -> MathUtil.applyDeadband(-controller.getLeftY(), 0.05),
            () -> MathUtil.applyDeadband(-controller.getLeftX(), 0.05),
            () -> MathUtil.applyDeadband(-controller.getRightX(), 0.05)));
    // TODO: Move deadband to constants file

    controller
        .start()
        .onTrue(
            new InstantCommand(
                () ->
                    RobotConfig.drive.setFieldOrientedDrive(
                        !RobotConfig.drive.isFieldOrientedDrive())));

    controller.back().onTrue(new InstantCommand(() -> RobotConfig.drive.resetOdometry()));

    /*
    // run arm at 4 volts
    controller
        .y()
        .whileTrue(Commands.startEnd(() -> arm.runVoltage(4), () -> arm.runVoltage(0), arm));
    // () -> arm.runVoltage(ArmVoltsEntry.getDouble(0.0)), () -> arm.runVoltage(0), arm));
    controller
        .x()
        .whileTrue(Commands.startEnd(() -> arm.runVoltage(-4), () -> arm.runVoltage(0), arm));
    */

    controller.b().whileTrue(new ArmToPositionDebug(arm));
  }

  public Command getAutonomousCommand() {
    if (autoChooser != null) {
      return autoChooser.getSelected();
    } else {
      return null;
    }
  }

  public void commandsToShuffleboard() {
    // SmartDashboard.putData(new ArmToPosition(arm));
    ShuffleboardTab armTab = Shuffleboard.getTab("Arm");
    armTab.add("armToPosition", new ArmToPositionDebug(arm)).withWidget(BuiltInWidgets.kCommand);
  }
}
