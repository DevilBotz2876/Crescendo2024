package frc.robot.commands;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.ArmSubsystem;
import java.util.Map;

public class ArmToPosition extends Command {
  ArmSubsystem arm;
  ShuffleboardTab tab;
  GenericEntry degreesEntry;

  public ArmToPosition(ArmSubsystem arm) {
    this.arm = arm;

    tab = Shuffleboard.getTab("Arm");

    degreesEntry =
        tab.add("Degrees Setpoint", 0.0)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", 0, "max", 110))
            .getEntry();

    addRequirements(arm);
  }

  @Override
  public void initialize() {
    // System.out.println("armToPos init: "+degreesEntry.getDouble(0));
  }

  @Override
  public void execute() {
    double setpoint = degreesEntry.getDouble(1.0);
    // System.out.println("armToPos setpoint: "+setpoint);
    SmartDashboard.putNumber("ShuffleBoard/Arm/Degree", setpoint);
    // Checks the degree Entry for the degree setpoint and sets the degree setpoint of motor
    arm.setAngle(setpoint);
  }

  @Override
  public void end(boolean interrupted) {
    arm.runVoltage(0);
    // arm.setAngle(5);
  }
}
