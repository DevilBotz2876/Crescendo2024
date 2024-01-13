// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.DriveBaseCommand;
import frc.robot.subsystems.drive.DriveBase;
import frc.robot.subsystems.drive.DriveSwerve;
import frc.robot.subsystems.drive.DriveTank;

public class RobotContainer {
  public final XboxController controller;
  public final DriveBase drivetrain;

  public RobotContainer() {
    controller = new XboxController(0);

    boolean swerveBot = false;
    boolean tankBot = false;

    if (swerveBot) {
      drivetrain = new DriveSwerve();
    } else if (tankBot) {
      drivetrain = new DriveTank();
    } else {
      drivetrain = new DriveBase();
    }

    configureBindings();
  }

  private void configureBindings() {
    drivetrain.setDefaultCommand(
        new DriveBaseCommand(
            drivetrain, () -> -controller.getLeftY(), () -> -controller.getLeftX()));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
