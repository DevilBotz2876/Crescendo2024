package frc.robot.commands.assist;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.Intake;

public class IndexPiece extends Command {
  public IndexPiece(Intake noteIntake) {
    this.intake = noteIntake;

    addRequirements((SubsystemBase) noteIntake);
  }

  Intake intake;

  @Override
  public void initialize() {
    intake.setVoltage(6);
    intake.enable();
  }

  @Override
  public void execute() {
    if (intake.isPieceDetected(true)) {
      intake.setVoltage(2);
    }
    intake.enable();
  }

  @Override
  public void end(boolean interrupted) {
    intake.disable();
  }

  @Override
  public boolean isFinished() {
    return intake.isPieceDetected(true) && intake.isPieceDetected(false);
  }
}
