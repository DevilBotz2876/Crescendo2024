package frc.robot.commands.assist;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.intake.Intake;
import edu.wpi.first.networktables.NetworkTableInstance;

public class PrepareForIntake extends SequentialCommandGroup {
  // TODO:  read arm angle and intake voltage from GUI
  NetworkTable assistGUI = NetworkTableInstance.getDefault().getTable("Shuffleboard/Assist");
  public PrepareForIntake(Arm arm, Intake intake) {
    addCommands(new InstantCommand(() -> arm.setAngle(assistGUI.getEntry("Intake Angle").getDouble(1))));
    addCommands(new InstantCommand(() -> intake.setVoltage(assistGUI.getEntry("Intake Volts").getDouble(6))));
  }
  
  
  
}