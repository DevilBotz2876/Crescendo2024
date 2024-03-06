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
  DoubleSupplier indexVoltage = null;
  double targetIntakeVoltage;
  double targetIndexVoltage;

  public IndexPiece(Intake intake, DoubleSupplier intakeVoltage, DoubleSupplier indexVoltage) {
    this.intake = intake;
    this.intakeVoltage = intakeVoltage;
    this.indexVoltage = indexVoltage;

    addRequirements((Subsystem) intake);
  }

  public IndexPiece(Intake intake) {
    this(intake, null, null);
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

    if (indexVoltage != null) {
      targetIndexVoltage = indexVoltage.getAsDouble();
    } else {
      targetIndexVoltage = IntakeConstants.indexSpeedInVolts;
    }
  }

  @Override
  public void execute() {
    if (!intake.isPieceShooterDetected()) {
      intake.runVoltage(targetIntakeVoltage);
    }
  }

  @Override
  public void end(boolean interrupted) {
    intake.runVoltage(0);
    if (Constants.debugCommands) {
      System.out.println("  END: " + this.getClass().getSimpleName());
    }
  }

  @Override
  public boolean isFinished() {
    return intake.isPieceShooterDetected();
  }
}
