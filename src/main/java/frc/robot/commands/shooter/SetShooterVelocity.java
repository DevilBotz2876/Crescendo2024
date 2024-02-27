package frc.robot.commands.shooter;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.RobotConfig.ShooterConstants;
import frc.robot.subsystems.shooter.Shooter;
import java.util.function.DoubleSupplier;

public class SetShooterVelocity extends Command {
  Shooter shooter;
  DoubleSupplier velocityRPM;

  public SetShooterVelocity(Shooter shooter, DoubleSupplier velocityRPM) {
    this.shooter = shooter;
    this.velocityRPM = velocityRPM;
    addRequirements((SubsystemBase) shooter);
  }

  @Override
  public void initialize() {
    System.out.println(
        "START: " + this.getClass().getSimpleName() + " velocity: " + velocityRPM.getAsDouble());
    shooter.runVelocity(velocityRPM.getAsDouble());
  }

  @Override
  public void execute() {}

  @Override
  public boolean isFinished() {

    if (Math.abs(
            Units.radiansPerSecondToRotationsPerMinute(shooter.getCurrentSpeed())
                - velocityRPM.getAsDouble())
        <= ShooterConstants.pidVelocityErrorInRPMS) {
      return true;
    }

    return false;
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("  END: " + this.getClass().getSimpleName());
  }
}
