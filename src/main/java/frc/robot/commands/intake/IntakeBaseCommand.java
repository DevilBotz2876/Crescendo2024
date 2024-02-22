package frc.robot.commands.intake;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.config.RobotConfig.IntakeConstants;
import frc.robot.subsystems.intake.Intake;
import java.util.function.BooleanSupplier;

public class IntakeBaseCommand extends Command {
  Intake intake;
  BooleanSupplier inEnable;
  BooleanSupplier outEnable;
  NetworkTable assistGUI = NetworkTableInstance.getDefault().getTable("Shuffleboard/Commands");
  NetworkTableEntry voltsEntry = assistGUI.getEntry("Intake: Volts");

  public IntakeBaseCommand(Intake intake, BooleanSupplier inEnable, BooleanSupplier outEnable) {
    this.intake = intake;
    this.inEnable = inEnable;
    this.outEnable = outEnable;

    addRequirements((Subsystem) intake);
  }

  @Override
  public void execute() {
    double volts = voltsEntry.getDouble(IntakeConstants.defaultSpeedInVolts);
    boolean in = this.inEnable.getAsBoolean();
    boolean out = this.outEnable.getAsBoolean();

    // Turns off motors if No/All bumpers
    if (in && out) {
      // Disable the intake motors
      intake.runVoltage(0);
    } else if (in == false && out == false) {
      intake.runVoltage(0);
    } else if (in) { // Motors on (IN) if right bumper pressed
      intake.runVoltage(volts);
    } else if (out) { // Motors on (Out) if right bumper pressed
      intake.runVoltage(volts * -1);
    } else { // Disable the intake motors
      intake.runVoltage(0);
    }
  }
}
