package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.config.RobotConfig.IntakeConstants;
import frc.robot.subsystems.intake.IntakeBase;
import java.util.function.BooleanSupplier;

public class IntakeBaseCommand extends Command {
  IntakeBase intake;
  BooleanSupplier inEnable;
  BooleanSupplier outEnable;
  NetworkTable assistGUI = NetworkTableInstance.getDefault().getTable("Shuffleboard/Assist");
  NetworkTableEntry voltsEntry = assistGUI.getEntry("Intake Piece Volts");

  public IntakeBaseCommand(IntakeBase intake, BooleanSupplier inEnable, BooleanSupplier outEnable) {
    this.intake = intake;
    this.inEnable = inEnable;
    this.outEnable = outEnable;

    addRequirements(intake);
  }

  @Override
  public void execute() {

    // Turns off motors if No/All bumpers
    if (inEnable.getAsBoolean() == true && outEnable.getAsBoolean() == true) {
      // Disable the intake motors
      intake.setVoltage(0);
      // System.out.println(outEnable.getAsBoolean());
    } else if (inEnable.getAsBoolean() == false && outEnable.getAsBoolean() == false) {
      intake.setVoltage(0);
      // System.out.println(outEnable.getAsBoolean());
    } else if (inEnable.getAsBoolean() == true) { // Motors on (IN) if right bumper pressed
      intake.setVoltage(voltsEntry.getDouble(IntakeConstants.intakeSpeedInVolts));
    } else if (outEnable.getAsBoolean() == true) { // Motors on (Out) if right bumper pressed
      intake.setVoltage(voltsEntry.getDouble(IntakeConstants.intakeSpeedInVolts) * -1);
    } else { // Disable the intake motors
      intake.setVoltage(0);
    }
  }
}
