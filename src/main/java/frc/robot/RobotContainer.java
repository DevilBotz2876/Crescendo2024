// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.IntakeBaseCommand;
import frc.robot.subsystems.intake.IntakeBase;
import frc.robot.subsystems.intake.IntakeIOSim;

public class RobotContainer {
  private final CommandXboxController controller;// = new CommandXboxController(0);
  public final IntakeBase intake;

  public RobotContainer() {
    controller = new CommandXboxController(0);
    
    boolean swerveBot = false;
    boolean tankBot = false;

    if (swerveBot) {
      intake = null;
    } else if (tankBot) {
      intake = null;
    } else {
      intake = new IntakeBase(new IntakeIOSim());
    }

    configureBindings();
  }

  private void configureBindings() {
    intake.setDefaultCommand(new IntakeBaseCommand(intake, () -> controller.rightBumper().getAsBoolean(), () -> controller.leftBumper().getAsBoolean() ));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
