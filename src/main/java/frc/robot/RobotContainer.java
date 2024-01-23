// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
//import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.drive.DriveCommand;
import frc.robot.subsystems.drive.DriveSwerveYAGSL;

public class RobotContainer {
  CommandXboxController controller = new CommandXboxController(0);
  DriveSwerveYAGSL drive = new DriveSwerveYAGSL();

   //private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    configureBindings();
    //autoChooser = AutoBuilder.buildAutoChooser("Mobility Auto");
    // SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void configureBindings() {
    drive.setDefaultCommand(
        new DriveCommand(
            drive,
            () -> MathUtil.applyDeadband(-controller.getLeftY(), 0.01),
            () -> MathUtil.applyDeadband(-controller.getLeftX(), 0.01),
            () -> MathUtil.applyDeadband(-controller.getRightX(), 0.01)));
    // TODO: Move deadband to constants file

    controller
        .start()
        .onTrue(
            new InstantCommand(() -> drive.setFieldOrientedDrive(!drive.isFieldOrientedDrive())));
  }

  public Command getAutonomousCommand() {
    return new PathPlannerAuto("Shoot Speaker Amp Side and Intake Amp Note Auto");
  }

}
