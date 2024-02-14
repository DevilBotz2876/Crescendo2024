// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.IntakeBaseCommand;
import frc.robot.commands.ShooterEnable;
import frc.robot.commands.drive.DriveCommand;
import frc.robot.config.RobotConfig;
import frc.robot.config.RobotConfigInferno;
import frc.robot.config.RobotConfigPhoenix;
import frc.robot.config.RobotConfigSherman;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

public class RobotContainer {
  public final CommandXboxController controller;
  public final RobotConfig robotConfig;
  private SendableChooser<Command> autoChooser = null;
  private static final String robotNameKey = "Robot Name";

  private final LoggedDashboardNumber shooterSpeedInput =
      new LoggedDashboardNumber("Shooter Speed", 1000.0);

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
    }

    configureBindings();
    // shooterSysIdBindings();
    // driveSysIdBindings();
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
  }

  public Command getAutonomousCommand() {
    if (autoChooser != null) {
      return autoChooser.getSelected();
    } else {
      return null;
    }
  }
}
