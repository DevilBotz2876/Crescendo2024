package frc.robot.commands.assist;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ArmToPosition;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.intake.Intake;

public class PrepareForIntake extends SequentialCommandGroup {
  // TODO:  read arm angle and intake voltage from GUI
  NetworkTable assistGUI = NetworkTableInstance.getDefault().getTable("Shuffleboard/Assist");

  // Sets the Arm to the ideal angle for intake
  // Turns the intake motor on
  public PrepareForIntake(Arm arm, Intake intake) {
    addCommands(
        new ArmToPosition((ArmSubsystem) arm, assistGUI.getEntry("Intake Angle").getDouble(1)));
    addCommands(new IndexPiece(intake));
  }
}
