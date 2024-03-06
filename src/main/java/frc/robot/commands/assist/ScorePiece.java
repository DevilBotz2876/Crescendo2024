package frc.robot.commands.assist;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.RobotConfig.IntakeConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import java.util.function.DoubleSupplier;

public class ScorePiece extends Command {
  Intake intake;
  Shooter shooter;
  DoubleSupplier intakeFeedVoltage;

  // Turn on intake (to feed piece into shooter)
  // Wait for piece to be shot out
  // Turn off intake
  // Turn off shooter
  public ScorePiece(Intake intake, Shooter shooter) {
    this(intake, shooter, () -> IntakeConstants.feedSpeedInVolts);
  }

  public ScorePiece(Intake intake, Shooter shooter, DoubleSupplier intakeFeedVoltage) {
    this.intake = intake;
    this.shooter = shooter;
    this.intakeFeedVoltage = intakeFeedVoltage;

    addRequirements((SubsystemBase) intake);
    addRequirements((SubsystemBase) shooter);
  }

  @Override
  public void initialize() {
    System.out.println("START: " + this.getClass().getSimpleName());
    intake.runVoltage(intakeFeedVoltage.getAsDouble());
  }

  @Override
  public boolean isFinished() {
    return !intake.isPieceShooterDetected();
  }

  @Override
  public void end(boolean interrupted) {
    intake.runVoltage(0);
    shooter.runVelocity(0);
    System.out.println("  END: " + this.getClass().getSimpleName());
  }
}
