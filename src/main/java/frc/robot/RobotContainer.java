// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.IntakeBaseCommand;
import frc.robot.commands.ShooterEnable;
import frc.robot.commands.drive.DriveCommand;
import frc.robot.subsystems.drive.DriveBase;
import frc.robot.subsystems.drive.DriveSwerveYAGSL;
import frc.robot.subsystems.intake.IntakeBase;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOTalonSRX;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.ShooterIOSparkMax;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

public class RobotContainer {
  public final CommandXboxController controller;
  public final ShooterSubsystem shooter;
  public final IntakeBase intake;
  public final DriveBase drive;
  private SendableChooser<Command> autoChooser = null;
  private static final String robotNameKey = "Robot Name";

  public enum RobotModel {
    PHOENIX, // Practice Swerve Bot
    SHERMAN, // Practice Tank Bot
  }

  public enum DriveType {
    NONE,
    SWERVE,
    TANK
  }

  private final LoggedDashboardNumber shooterSpeedInput =
      new LoggedDashboardNumber("Shooter Speed", 1000.0);

  public RobotContainer() {
    RobotModel model = RobotModel.PHOENIX; // Default if "Robot Name" not found in preferences
    String robotName = "UNKNOWN";

    controller = new CommandXboxController(0);

    boolean hasIntake = false;
    boolean hasShooter = false;
    DriveType driveType = DriveType.NONE;

    Preferences.initString(robotNameKey, robotName);
    robotName = Preferences.getString(robotNameKey, robotName);
    System.out.println("Loading Settings for Robot Name = " + robotName);
    switch (robotName) {
      case "PHOENIX":
        model = RobotModel.PHOENIX;
        break;
      case "SHERMAN":
        model = RobotModel.SHERMAN;
        break;
      case "UNKNOWN":
      default:
    }

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
      shooter = new ShooterSubsystem(new ShooterIOSparkMax(2), new ShooterIOSparkMax(1));
    } else {
      shooter =
          new ShooterSubsystem(
              new ShooterIOSim(ShooterIOSim.ShooterId.SHOOTER_TOP),
              new ShooterIOSim(ShooterIOSim.ShooterId.SHOOTER_BOTTOM));
    }

    if (hasIntake) {
      intake = new IntakeBase(new IntakeIOTalonSRX());
    } else {
      intake = new IntakeBase(new IntakeIOSim());
    }

    switch (driveType) {
      case SWERVE:
        drive = new DriveSwerveYAGSL();
        autoChooser = AutoBuilder.buildAutoChooser("Mobility Auto");
        SmartDashboard.putData("Auto Chooser", autoChooser);
        break;
      case TANK:
        drive = new DriveBase();
        break;
      default:
        drive = new DriveBase();
    }

    configureBindings();
    // shooterSysIdBindings();
  }

  private void shooterSysIdBindings() {
    controller.a().whileTrue(shooter.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    controller.b().whileTrue(shooter.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    controller.x().whileTrue(shooter.sysIdDynamic(SysIdRoutine.Direction.kForward));
    controller.y().whileTrue(shooter.sysIdDynamic(SysIdRoutine.Direction.kReverse));
  }

  private void configureBindings() {
    // shooter.setDefaultCommand(new InstantCommand(() -> shooter.disable(), shooter));

    controller.rightTrigger().whileTrue(new ShooterEnable(shooter));

    controller
        .a()
        .whileTrue(
            Commands.startEnd(
                () -> shooter.runVelocity(shooterSpeedInput.get()),
                () -> shooter.runVelocity(0),
                shooter));

    intake.setDefaultCommand(
        new IntakeBaseCommand(
            intake,
            () -> controller.rightBumper().getAsBoolean(),
            () -> controller.leftBumper().getAsBoolean()));

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

  public Command getAutonomousCommand() {
    if (autoChooser != null) {
      return autoChooser.getSelected();
    } else {
      return null;
    }
  }
}
