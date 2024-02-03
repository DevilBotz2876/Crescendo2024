// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.IntakeBaseCommand;
import frc.robot.commands.ShooterEnable;
import frc.robot.commands.drive.DriveCommand;
import frc.robot.subsystems.drive.DriveBase;
import frc.robot.subsystems.drive.DriveSwerveYAGSL;
import frc.robot.subsystems.drive.DriveTrain;
import frc.robot.subsystems.intake.IntakeBase;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOSparkMax;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.ShooterIOSparkMax;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class RobotContainer {
  public final CommandXboxController controller;
  public final ShooterSubsystem shooter;
  public final IntakeBase intake;
  public final DriveBase drive;

  public enum RobotModel {
    PHOENIX, // Practice Swerve Bot
    SHERMAN, // Practice Tank Bot
  }

  public enum DriveType {
    NONE,
    SWERVE,
    TANK
  }

  public RobotContainer() {
    RobotModel model = RobotModel.PHOENIX;

    controller = new CommandXboxController(0);

    boolean hasIntake = false;
    boolean hasShooter = false;
    DriveType driveType = DriveType.NONE;

    switch (model) {
      case PHOENIX:
        driveType = DriveType.SWERVE;
        break;
      case SHERMAN:
        driveType = DriveType.TANK;
        hasIntake = true;
        hasShooter = true;
        break;
      default:
    }

    if (hasShooter) {
      shooter = new ShooterSubsystem(new ShooterIOSparkMax());
    } else {
      shooter = new ShooterSubsystem(new ShooterIOSim());
    }

    if (hasIntake) {
      intake = new IntakeBase(new IntakeIOSparkMax());
    } else {
      intake = new IntakeBase(new IntakeIOSim());
    }

    switch (driveType) {
      case SWERVE:
        drive = new DriveSwerveYAGSL();
        break;
      case TANK:
        drive = new DriveTrain();
        break;
      default:
        drive = null;
    }

    configureBindings();
  }

  private void configureBindings() {
    shooter.setDefaultCommand(new InstantCommand(() -> shooter.disable(), shooter));
    controller.rightTrigger().whileTrue(new ShooterEnable(shooter));

    intake.setDefaultCommand(
        new IntakeBaseCommand(
            intake,
            () -> controller.rightBumper().getAsBoolean(),
            () -> controller.leftBumper().getAsBoolean()));

    if (drive != null) {
      drive.setDefaultCommand(
          new DriveCommand(
              drive,
              () -> MathUtil.applyDeadband(-controller.getLeftY(), 0.05),
              () -> MathUtil.applyDeadband(-controller.getLeftX(), 0.05),
              () -> MathUtil.applyDeadband(-controller.getRightX(), 0.05)));
      // TODO: Move deadband to constants file

      controller
          .start()
          .onTrue(
              new InstantCommand(() -> drive.setFieldOrientedDrive(!drive.isFieldOrientedDrive())));

      controller.back().onTrue(new InstantCommand(() -> drive.resetOdometry()));
    }
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
