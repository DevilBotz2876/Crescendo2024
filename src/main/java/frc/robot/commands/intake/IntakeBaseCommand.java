package frc.robot.commands.intake;

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
    double volts = voltsEntry.getDouble(IntakeConstants.intakeSpeedInVolts);
    boolean in = inEnable.getAsBoolean();
    boolean out = outEnable.getAsBoolean();
    // Turns off motors if No/All bumpers
    if (in && out) {
      // Disable the intake motors
      intake.setVoltage(0);
    } else if (in == false && out == false) {
      intake.setVoltage(0);
    } else if (in) { // Motors on (IN) if right bumper pressed
      intake.setVoltage(volts);
    } else if (out) { // Motors on (Out) if right bumper pressed
      intake.setVoltage(volts * -1);
    } else { // Disable the intake motors
      intake.setVoltage(0);
    }
  }
}
