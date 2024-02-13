package frc.robot.commands;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeBase;
import java.util.Map;
import java.util.function.BooleanSupplier;

public class IntakeBaseCommand extends Command {
  IntakeBase intake;
  BooleanSupplier inEnable;
  GenericEntry voltsEntry;
  GenericEntry degreeEntry;
  ShuffleboardTab tab;
  BooleanSupplier outEnable;

  public IntakeBaseCommand(IntakeBase intake, BooleanSupplier inEnable, BooleanSupplier outEnable) {
    this.intake = intake;
    this.inEnable = inEnable;
    this.outEnable = outEnable;

    addRequirements(intake);
    tab = Shuffleboard.getTab("Assist");
    // Create volt entry under Shooter tab as a number sider with min = -1 and max = 1
    voltsEntry =
        tab.add("Intake Volts", 0)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", 0, "max", 12))
            .getEntry();
    voltsEntry.setValue(0.0);
    tab = Shuffleboard.getTab("Assist");
    degreeEntry = 
        tab.add("Intake Angle", 1)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", 0, "max", 180 ))
            .getEntry();
    degreeEntry.setValue(1);

  }

  @Override
  public void initialize() {
    intake.disable();
  }

  @Override
  public void execute() {

    // Turns off motors if No/All bumpers
    if (inEnable.getAsBoolean() == true && outEnable.getAsBoolean() == true) {
      // Disable the intake motors
      intake.disable();
      // System.out.println(outEnable.getAsBoolean());
    } else if (inEnable.getAsBoolean() == false && outEnable.getAsBoolean() == false) {
      intake.disable();
      // System.out.println(outEnable.getAsBoolean());
    } else if (inEnable.getAsBoolean() == true) { // Motors on (IN) if right bumper pressed
      intake.setVoltage(voltsEntry.getDouble(0.0));
      System.out.println(outEnable.getAsBoolean());

      // Enable motors, It has to be called regularly for voltage compensation to work properly
      intake.enable();
    } else if (outEnable.getAsBoolean() == true) { // Motors on (Out) if right bumper pressed
      intake.setVoltage(voltsEntry.getDouble(0.0) * -1);
      System.out.println(outEnable.getAsBoolean());
      // Enable motors, It has to be called regularly for voltage compensation to work properly
      intake.enable();
    } else { // Disable the intake motors
      intake.disable();
    }
    // System.out.println(outEnable.getAsBoolean());
  }
}
