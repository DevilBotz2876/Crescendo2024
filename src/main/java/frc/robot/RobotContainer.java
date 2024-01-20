// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.IntakeBaseCommand;
import frc.robot.commands.ShooterEnable;
import frc.robot.subsystems.intake.IntakeBase;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class RobotContainer {
  public final CommandXboxController controller;
  public final ShooterSubsystem shooter;
  public final IntakeBase intake;

  public RobotContainer() {
    controller = new CommandXboxController(0);

    boolean swerveBot = false;
    boolean tankBot = false;

    if (swerveBot) {
      shooter = null;
      intake = null;
    } else if (tankBot) {
      shooter = null;
      intake = null;
    } else {
      shooter = new ShooterSubsystem(new ShooterIOSim());
      intake = new IntakeBase(new IntakeIOSim());
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
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
