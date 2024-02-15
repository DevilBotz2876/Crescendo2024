package frc.robot.commands;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.arm.ArmSubsystem;
import java.util.Map;

/**
 * This class uses a widget on Shuffleboard to control the arm setpoint. It is meant to be used for
 * debug/test/bring type work.
 */
public class ArmToPositionDebug extends Command {
  ArmSubsystem arm;
  ShuffleboardTab tab;
  GenericEntry degreesEntry;

  public ArmToPositionDebug(ArmSubsystem arm) {
    this.arm = arm;

    tab = Shuffleboard.getTab("Arm");

    degreesEntry =
        tab.add("Degrees Setpoint", 0.0)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", Constants.armMinDegrees, "max", Constants.armMaxDegrees))
            .getEntry();

    addRequirements(arm);
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
    arm.runVoltage(0);
  }
}
