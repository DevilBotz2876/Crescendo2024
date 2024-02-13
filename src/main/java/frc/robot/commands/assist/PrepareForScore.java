package frc.robot.commands.assist;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.shooter.Shooter;

public class PrepareForScore extends SequentialCommandGroup {
  // TODO:  read arm angle and shooter velocity from GUI
  NetworkTable assistGUI = NetworkTableInstance.getDefault().getTable("Shuffleboard/Assist");
  public PrepareForScore(Arm arm, Shooter shooter) {
    addCommands(new InstantCommand(() -> arm.setAngle(assistGUI.getEntry("Shooter Angle").getDouble(100))));
    addCommands(new InstantCommand(() -> shooter.runVelocity(assistGUI.getEntry("Shooter Velocity").getDouble(100))));
  }
}
