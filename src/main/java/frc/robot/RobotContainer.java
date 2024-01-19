// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.drive.DriveCommand;
import frc.robot.subsystems.drive.DriveSwerveYAGSL;

public class RobotContainer {
  CommandXboxController controller = new CommandXboxController(0);
  DriveSwerveYAGSL drive = new DriveSwerveYAGSL();

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    drive.setDefaultCommand(
        new DriveCommand(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    controller
        .start()
        .onTrue(
            new InstantCommand(() -> drive.setFieldOrientedDrive(!drive.isFieldOrientedDrive())));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
