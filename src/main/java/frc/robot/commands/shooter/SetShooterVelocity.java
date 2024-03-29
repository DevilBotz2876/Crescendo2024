package frc.robot.commands.shooter;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.config.RobotConfig.ShooterConstants;
import frc.robot.subsystems.shooter.Shooter;
import java.util.function.DoubleSupplier;

public class SetShooterVelocity extends Command {
  Shooter shooter;
  DoubleSupplier velocityRPM;
  double targetVelocityRPM;
  Timer timer = new Timer();

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
    timer.restart();
    targetVelocityRPM = velocityRPM.getAsDouble();
    shooter.runVelocity(targetVelocityRPM);
  }

  @Override
  public boolean isFinished() {
    if (Math.abs(
            Units.radiansPerSecondToRotationsPerMinute(shooter.getCurrentSpeed())
                - targetVelocityRPM)
        <= ShooterConstants.pidVelocityErrorInRPMS) {
      if (timer.get() >= ShooterConstants.pidSettlingTimeInSeconds) {
        return true;
      }
    } else {
      timer.reset();
    }

    return false;
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
