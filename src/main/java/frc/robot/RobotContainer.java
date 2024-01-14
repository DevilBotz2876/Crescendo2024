// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.IntakeBaseCommand;
import frc.robot.subsystems.intake.IntakeBase;

public class RobotContainer {
  public final XboxController controller;
  public final IntakeBase intake;

  public RobotContainer() {
    controller = new XboxController(0);

    boolean swerveBot = false;
    boolean tankBot = false;

    if (swerveBot) {
      intake = null;
    } else if (tankBot) {
      intake = null;
    } else {
      intake = new IntakeBase();
    }

    configureBindings();
  }

  private void configureBindings() {
    intake.setDefaultCommand(new IntakeBaseCommand(intake, () -> controller.getRightBumper(), () -> controller.getLeftBumper() ));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
