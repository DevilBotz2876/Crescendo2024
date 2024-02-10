package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;

import java.util.Map;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.ArmSubsystem;

public class ArmToPosition extends Command {
    ArmSubsystem arm;
  ShuffleboardTab tab;
  GenericEntry degreesEntry;

  public ArmToPosition(ArmSubsystem arm) {
    this.arm = arm;

    // tab = Shuffleboard.getTab("Arm");

    // degreesEntry =
    //     tab.add("Degrees Setpoint", 0.0)
    //         .withWidget(BuiltInWidgets.kNumberSlider)
    //         .withProperties(Map.of("min", 0, "max", 110))
    //         .getEntry();

    // degreesEntry.setValue(0.0);

    

    addRequirements(arm);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    //SmartDashboard.putNumber("ShuffleBoard/Arm/Degree", degreesEntry.getDouble(1.0));
    // Checks the degree Entry for the degree setpoint and sets the degree setpoint of motor 
    //arm.setAngle(degreesEntry.getDouble(1.0));
  }

  @Override
  public void end(boolean interrupted) {
    arm.runVoltage(0);
  }
}
