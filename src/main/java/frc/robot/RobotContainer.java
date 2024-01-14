// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.ShooterBaseCommand;
import frc.robot.subsystems.shooter.ShooterBase;

public class RobotContainer {
  public final XboxController controller;
  public final ShooterBase shooter;

  public RobotContainer() {
    controller = new XboxController(0);

    boolean swerveBot = false;
    boolean tankBot = false;

    if (swerveBot) {
      shooter = null;
    } else if (tankBot) {
      shooter = null;
    } else {
      shooter = new ShooterBase();
    }


    configureBindings();
  }

  private void configureBindings() {
    shooter.setDefaultCommand(new ShooterBaseCommand(shooter, () -> controller.getRightTriggerAxis() ));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
