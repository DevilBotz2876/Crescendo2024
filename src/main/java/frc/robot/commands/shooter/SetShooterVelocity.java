package frc.robot.commands.shooter;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.RobotConfig.ShooterConstants;
import frc.robot.subsystems.shooter.Shooter;
import java.util.function.DoubleSupplier;

public class SetShooterVelocity extends Command {
  Shooter shooter;
  DoubleSupplier velocityRPM;
  double timeMS;

  public SetShooterVelocity(Shooter shooter, DoubleSupplier velocityRPM) {
    this.shooter = shooter;
    this.velocityRPM = velocityRPM;
    addRequirements((SubsystemBase) shooter);
  }

  @Override
  public void initialize() {
    shooter.runVelocity(velocityRPM.getAsDouble());
  }

  @Override
  public void execute() {}

  @Override
  public boolean isFinished() {

    if (Units.radiansPerSecondToRotationsPerMinute(shooter.getCurrentSpeed())
            > velocityRPM.getAsDouble() - ShooterConstants.pidVelocityErrorInRPMS
        && Units.radiansPerSecondToRotationsPerMinute(shooter.getCurrentSpeed())
            < velocityRPM.getAsDouble() + ShooterConstants.pidVelocityErrorInRPMS) {
      timeMS += 20.0;
      if (timeMS == 1000) {
        SmartDashboard.putBoolean("Shooter/SetShooterVelocity/isFinished", true);
        return true;
      }
    } else {
      timeMS = 0.0;
    }
    SmartDashboard.putBoolean("Shooter/SetShooterVelocity/isFinished", false);
    return false;
  }
}
