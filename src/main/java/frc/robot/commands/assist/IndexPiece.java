package frc.robot.commands.assist;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.config.RobotConfig.IntakeConstants;
import frc.robot.subsystems.intake.Intake;
import java.util.function.DoubleSupplier;

public class IndexPiece extends Command {
  Intake intake;
  DoubleSupplier intakeVoltage = null;
  double targetIntakeVoltage;

  public IndexPiece(Intake intake, DoubleSupplier intakeVoltage) {
    this.intake = intake;
    this.intakeVoltage = intakeVoltage;

    addRequirements((Subsystem) intake);
  }

  public IndexPiece(Intake intake) {
    this(intake, null);
  }

  @Override
  public void initialize() {
    if (Constants.debugCommands) {
      System.out.println("START: " + this.getClass().getSimpleName());
    }
    if (intakeVoltage != null) {
      targetIntakeVoltage = intakeVoltage.getAsDouble();
    } else {
      targetIntakeVoltage = IntakeConstants.defaultSpeedInVolts;
    }
  }

  @Override
  public void execute() {
    if (!intake.isPieceDetected()) {
      intake.runVoltage(targetIntakeVoltage);
    }
  }

  @Override
  public void end(boolean interrupted) {

    if (!interrupted) {
      intake.runVoltage(0);
    }
    if (Constants.debugCommands) {
      System.out.println("  END: " + this.getClass().getSimpleName());
    }
  }

  @Override
  public boolean isFinished() {
    return intake.isPieceDetected();
  }
}
