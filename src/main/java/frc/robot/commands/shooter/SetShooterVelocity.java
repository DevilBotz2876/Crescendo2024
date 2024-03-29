package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.Shooter;
import java.util.function.DoubleSupplier;

public class SetShooterVelocity extends Command {
  Shooter shooter;
  DoubleSupplier velocityRPM;
  double targetVelocityRPM;

  public SetShooterVelocity(Shooter shooter, DoubleSupplier velocityRPM) {
    this.shooter = shooter;
    this.velocityRPM = velocityRPM;
    addRequirements((SubsystemBase) shooter);
  }

  @Override
  public void initialize() {
    if (Constants.debugCommands) {
      System.out.println(
          "START: " + this.getClass().getSimpleName() + " velocity: " + velocityRPM.getAsDouble());
    }
    targetVelocityRPM = velocityRPM.getAsDouble();
    shooter.runVelocity(targetVelocityRPM);
  }

  @Override
  public boolean isFinished() {
    return shooter.isAtSetpoint();
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
