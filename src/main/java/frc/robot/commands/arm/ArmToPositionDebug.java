package frc.robot.commands.arm;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.arm.Arm;

/**
 * This class uses a widget on Shuffleboard to control the arm setpoint. It is meant to be used for
 * debug/test/bring type work.
 */
public class ArmToPositionDebug extends Command {
  Arm arm;
  ShuffleboardTab tab;
  NetworkTable armGUI = NetworkTableInstance.getDefault().getTable("Shuffleboard/Arm");
  NetworkTableEntry degreesEntry = armGUI.getEntry("Degrees Setpoint");

  public ArmToPositionDebug(Arm arm) {
    this.arm = arm;

    addRequirements((SubsystemBase) arm);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    double setpoint = degreesEntry.getDouble(1.0);
    arm.setAngle(setpoint);
  }

  @Override
  public void end(boolean interrupted) {
    // arm.runVoltage(0);
  }
}
