// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.ArmToPositionDebug;
import frc.robot.commands.IntakeBaseCommand;
import frc.robot.commands.TestShooterAngle;
import frc.robot.commands.assist.IndexPiece;
import frc.robot.commands.assist.PrepareForIntake;
import frc.robot.commands.assist.PrepareForScore;
import frc.robot.commands.assist.ScorePiece;
import frc.robot.commands.drive.DriveCommand;
import frc.robot.config.RobotConfig;
import frc.robot.config.RobotConfigInferno;
import frc.robot.config.RobotConfigPhoenix;
import frc.robot.config.RobotConfigSherman;

public class RobotContainer {
  public final CommandXboxController controller;
  public final RobotConfig robotConfig;
  private static final String robotNameKey = "Robot Name";

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
        robotConfig = new RobotConfigInferno();
        // robotConfig = new RobotConfigSherman();
    }

    // SmartDashboard.putData("Subsystems/Arm", RobotConfig.arm);
    // SmartDashboard.putData(CommandScheduler.getInstance());

    ShuffleboardTab autoTab = Shuffleboard.getTab("Autonomous");
    // Create volt entry under Shooter tab as a number sider with min = -1 and max = 1
    autoTab
        .add("Auto Chooser", RobotConfig.autoChooser)
        .withWidget(BuiltInWidgets.kComboBoxChooser);

    configureBindings();
    // ArmSysIdBindings();
    // shooterSysIdBindings();
    // driveSysIdBindings();

    commandsToShuffleboard();
  }

  private void ArmSysIdBindings() {
    controller.a().whileTrue(RobotConfig.arm.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    controller.b().whileTrue(RobotConfig.arm.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    controller.x().whileTrue(RobotConfig.arm.sysIdDynamic(SysIdRoutine.Direction.kForward));
    controller.y().whileTrue(RobotConfig.arm.sysIdDynamic(SysIdRoutine.Direction.kReverse));
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
    // controller.rightTrigger().onTrue(new ScorePiece(RobotConfig.intake, RobotConfig.shooter));

    // controller.a().onTrue(new PrepareForIntake(RobotConfig.arm, RobotConfig.intake));

    // controller.b().onTrue(new PrepareForScore(RobotConfig.arm, RobotConfig.shooter));

    /*
    controller.rightTrigger().whileTrue(new ShooterEnable(RobotConfig.shooter));

    private final LoggedDashboardNumber shooterSpeedInput =
    new LoggedDashboardNumber("Shooter Speed", 1000.0);

    controller
        .a()
        .whileTrue(
            Commands.startEnd(
                () -> RobotConfig.shooter.runVelocity(shooterSpeedInput.get()),
                () -> RobotConfig.shooter.runVelocity(0),
                RobotConfig.shooter));
    */

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

    // controller
    //     .start()
    //     .onTrue(
    //         new InstantCommand(
    //             () ->
    //                 RobotConfig.drive.setFieldOrientedDrive(
    //                     !RobotConfig.drive.isFieldOrientedDrive())));

    // controller.back().onTrue(new InstantCommand(() -> RobotConfig.drive.resetOdometry()));

    /*
    // run arm at 4 volts
    controller
        .y()
        .whileTrue(Commands.startEnd(() -> RobotConfig.arm.runVoltage(4), () -> RobotConfig.arm.runVoltage(0), arm));
    // () -> arm.runVoltage(ArmVoltsEntry.getDouble(0.0)), () -> arm.runVoltage(0), arm));
    controller
        .x()
        .whileTrue(Commands.startEnd(() -> RobotConfig.arm.runVoltage(-4), () -> RobotConfig.arm.runVoltage(0), arm));
    */

    // controller.b().whileTrue(new ArmToPositionDebug(RobotConfig.arm));
    RobotConfig.arm.setDefaultCommand(new ArmToPositionDebug(RobotConfig.arm));
  }

  public Command getAutonomousCommand() {
    return RobotConfig.autoChooser.getSelected();
  }

  public void commandsToShuffleboard() {
    // SmartDashboard.putData(new ArmToPosition(arm));
    ShuffleboardTab armTab = Shuffleboard.getTab("SubsCommands");

    // Create a layout to hold commands
    ShuffleboardLayout commandLayout =
        armTab.getLayout("Commands", BuiltInLayouts.kList).withPosition(0, 0).withSize(2, 4);

    // Create a layout to hold subsystems
    ShuffleboardLayout subLayout =
        armTab.getLayout("Subsystems", BuiltInLayouts.kList).withPosition(2, 0).withSize(3, 4);

    commandLayout.add(new ArmToPositionDebug(RobotConfig.arm));
    commandLayout.add(new IndexPiece(RobotConfig.intake));
    commandLayout.add(new PrepareForIntake(RobotConfig.arm, RobotConfig.intake));
    commandLayout.add(new PrepareForScore(RobotConfig.arm, RobotConfig.shooter));
    commandLayout.add(new ScorePiece(RobotConfig.intake, RobotConfig.shooter));
    commandLayout.add(
        new TestShooterAngle(RobotConfig.shooter, RobotConfig.intake, RobotConfig.arm));

    subLayout.add(RobotConfig.arm);
    subLayout.add(RobotConfig.intake);
    subLayout.add(RobotConfig.shooter);
    subLayout.add(RobotConfig.drive);
  }
}
