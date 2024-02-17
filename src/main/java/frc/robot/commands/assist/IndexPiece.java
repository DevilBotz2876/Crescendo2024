package frc.robot.commands.assist;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.RobotConfig.IntakeConstants;
import frc.robot.subsystems.intake.Intake;

public class IndexPiece extends Command {
  Intake intake;
  NetworkTable assistGUI = NetworkTableInstance.getDefault().getTable("Shuffleboard/Assist");

  public IndexPiece(Intake noteIntake) {
    this.intake = noteIntake;

    addRequirements((SubsystemBase) noteIntake);
  }

  @Override
  public void execute() {
    if (intake.isPieceDetected(true)) {
      intake.setVoltage(
          assistGUI.getEntry("Index Piece Volts").getDouble(IntakeConstants.intakeSpeedInVolts));
    } else {
      intake.setVoltage(
          assistGUI.getEntry("Intake Piece Volts").getDouble(IntakeConstants.indexSpeedInVolts));
    }
  }

  @Override
  public void end(boolean interrupted) {
    intake.setVoltage(0);
  }

  @Override
  public boolean isFinished() {
    return intake.isPieceDetected(true) && intake.isPieceDetected(false);
  }
}
