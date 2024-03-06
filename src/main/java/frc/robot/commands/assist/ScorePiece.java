package frc.robot.commands.assist;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.RobotConfig.IntakeConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

public class ScorePiece extends Command {
  Intake intake;
  Shooter shooter;

  // Turn on intake (to feed piece into shooter)
  // Wait for piece to be shot out
  // Turn off intake
  // Turn off shooter
  public ScorePiece(Intake intake, Shooter shooter) {
    this.intake = intake;
    this.shooter = shooter;

    addRequirements((SubsystemBase) intake);
    addRequirements((SubsystemBase) shooter);
  }

  @Override
  public void initialize() {
    intake.runVoltage(IntakeConstants.feedSpeedInVolts);
  }

  @Override
  public boolean isFinished() {
    return !intake.isPieceShooterDetected();
  }

  @Override
  public void end(boolean interrupted) {
    intake.runVoltage(0);
    shooter.runVelocity(0);
  }
}
