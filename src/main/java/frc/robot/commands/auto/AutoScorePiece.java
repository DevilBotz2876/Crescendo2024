package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.config.RobotConfig.IntakeConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import java.util.function.DoubleSupplier;

public class AutoScorePiece extends Command {
  Intake intake;
  Shooter shooter;
  DoubleSupplier intakeFeedVoltage;

  // Turn on intake (to feed piece into shooter)
  // Wait for piece to be shot out
  // Turn off intake
  // Turn off shooter
  public AutoScorePiece(Intake intake, Shooter shooter) {
    this(intake, shooter, () -> IntakeConstants.defaultSpeedInVolts);
  }

  public AutoScorePiece(Intake intake, Shooter shooter, DoubleSupplier intakeFeedVoltage) {
    this.intake = intake;
    this.shooter = shooter;
    this.intakeFeedVoltage = intakeFeedVoltage;

    addRequirements((SubsystemBase) intake);
    addRequirements((SubsystemBase) shooter);
  }

  @Override
  public void initialize() {
    if (Constants.debugCommands) {
      System.out.println("START: " + this.getClass().getSimpleName());
    }
    intake.runVoltage(intakeFeedVoltage.getAsDouble());
  }

  @Override
  public boolean isFinished() {
    return !intake.isPieceDetected();
  }

  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      System.err.println("INTERRUPTED: " + this.getClass().getSimpleName());
    }

    if (Constants.debugCommands) {
      System.out.println("  END: " + this.getClass().getSimpleName());
    }
  }
}
