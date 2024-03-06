// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.config.RobotConfig;
import frc.robot.config.RobotConfigInferno;
import frc.robot.config.RobotConfigPhoenix;
import frc.robot.config.RobotConfigSherman;
import frc.robot.controls.DebugControls;
import frc.robot.controls.DriverControls;
import frc.robot.controls.PitControls;

public class RobotContainer {
  public final CommandXboxController controller;
  public final RobotConfig robotConfig;
  private static final String robotNameKey = "Robot Name";

  public RobotContainer() {
    String robotName = "UNKNOWN";
    controller = new CommandXboxController(0);

    Preferences.initString(robotNameKey, robotName);
    robotName = Preferences.getString(robotNameKey, robotName);
    System.out.println("Loading Settings for Robot Name = " + robotName);
    switch (robotName) {
      case "PHOENIX":
        robotConfig = new RobotConfigPhoenix();
        break;
      case "SHERMAN":
        robotConfig = new RobotConfigSherman();
        break;
      case "INFERNO":
        robotConfig = new RobotConfigInferno();
        break;
      case "UNKNOWN":
      default:
        /* If running simulation, put the robot config you want here */
        // robotConfig = new RobotConfigInferno();
        robotConfig = new RobotConfigPhoenix();
        // robotConfig = new RobotConfigSherman();
        // robotConfig = new RobotConfigStub();
    }

    configureBindings();
  }

  void configureBindings() {
    DriverControls.setupControls(controller);
    DriverControls.setupGUI();
    PitControls.setupGUI();
    DebugControls.setupGUI();
    //    SysIdControls.setupGUI();
  }

  public Command getAutonomousCommand() {
    return RobotConfig.autoChooser.getSelected();
  }
}
