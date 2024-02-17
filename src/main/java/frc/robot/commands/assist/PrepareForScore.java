package frc.robot.commands.assist;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.ArmToPosition;
import frc.robot.config.RobotConfig.ArmConstants;
import frc.robot.config.RobotConfig.ShooterConstants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.shooter.Shooter;

public class PrepareForScore extends ParallelCommandGroup {
  // TODO:  read arm angle and shooter velocity from GUI
  NetworkTable assistGUI = NetworkTableInstance.getDefault().getTable("Shuffleboard/Assist");

  // Moves arm to ideal angle for shooting
  // Turns on shooter to ideal speed for scoring
  public PrepareForScore(Arm arm, Shooter shooter) {
    addCommands(
        new ArmToPosition(
            (ArmSubsystem) arm,
            () ->
                assistGUI.getEntry("Shooter Angle").getDouble(ArmConstants.shooterAngleInDegrees)));
    addCommands(
        new InstantCommand(
            () ->
                shooter.runVelocity(
                    assistGUI
                        .getEntry("Shooter Velocity")
                        .getDouble(ShooterConstants.velocityInRPMs))));
  }
}
