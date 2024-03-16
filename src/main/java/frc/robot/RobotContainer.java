// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.config.RobotConfig;
import frc.robot.config.RobotConfigInferno;
import frc.robot.config.RobotConfigPhoenix;
import frc.robot.config.RobotConfigSherman;
import frc.robot.controls.DebugControls;
import frc.robot.controls.DriverControls;
import frc.robot.controls.PitControls;
import frc.robot.controls.SysIdControls;

public class RobotContainer {
  public final RobotConfig robotConfig;
  private static final String robotNameKey = "Robot Name";
  private Command autoCommand = null;

  public RobotContainer() {
    String robotName = "UNKNOWN";

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
    DriverControls.setupControls();
    PitControls.setupControls();
    DebugControls.setupControls();
    SysIdControls.setupGUI();
  }

  public Command getAutonomousCommand() {
    /*
     if (autoCommand == null) {
       autoCommand =
           new SequentialCommandGroup(
               RobotConfig.climber
                   .getPrepareClimberForMatchStartCommand()
                   .onlyIf(() -> DevilBotState.climberNeedsToBeZeroedAtStart), // Move climber down
               new ParallelCommandGroup(
                   RobotConfig.climber
                       .getAutoZeroCommand()
                       .onlyIf(
                           () ->
                               DevilBotState
                                   .climberNeedsToBeZeroedAtStart), // auto zero climber command
                   // while
                   // running selected auto
                   RobotConfig.autoChooser
                       .getSelected()
                       .andThen(
                           new ParallelCommandGroup(
                               RobotConfig.shooter.getTurnOffCommand(),
                               RobotConfig.intake.getTurnOffCommand()))),
               new InstantCommand(() -> DevilBotState.climberNeedsToBeZeroedAtStart = false),
               new ParallelCommandGroup(
                   RobotConfig.shooter.getTurnOffCommand(),
                   RobotConfig.intake
                       .getTurnOffCommand())); // Turn off shooter/intake after autonomous
     }
     return autoCommand;
    */
    return RobotConfig.autoChooser.getSelected();
  }
}
