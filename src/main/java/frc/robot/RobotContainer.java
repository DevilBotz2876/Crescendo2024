// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ShooterBaseCommand;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class RobotContainer {
  public final CommandXboxController controller;
  public final ShooterSubsystem shooter;

  public RobotContainer() {
    controller = new CommandXboxController(0);

    boolean swerveBot = false;
    boolean tankBot = false;

    if (swerveBot) {
      shooter = null;
    } else if (tankBot) {
      shooter = null;
    } else {
      shooter = new ShooterSubsystem(new ShooterIOSim());
    }

    shooter.setDefaultCommand(new InstantCommand(() -> shooter.disable(), shooter));

    configureBindings();
  }

  private void configureBindings() {
    controller.rightTrigger().onTrue(new ShooterBaseCommand(shooter, () -> true));
    controller.rightTrigger().onFalse(shooter.getDefaultCommand());
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
