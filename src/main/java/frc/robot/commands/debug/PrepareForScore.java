package frc.robot.commands.debug;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.arm.ArmToPosition;
import frc.robot.commands.shooter.SetShooterVelocity;
import frc.robot.config.RobotConfig.ArmConstants;
import frc.robot.config.RobotConfig.ShooterConstants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.shooter.Shooter;
import java.util.function.DoubleSupplier;

public class PrepareForScore extends ParallelCommandGroup {
  public PrepareForScore(
      Arm arm, Shooter shooter, DoubleSupplier armAngle, DoubleSupplier shooterVelocity) {
    addCommands(new ArmToPosition(arm, armAngle).withTimeout(ArmConstants.pidTimeoutInSeconds));
    addCommands(
        new SetShooterVelocity(shooter, shooterVelocity)
            .withTimeout(ShooterConstants.pidTimeoutInSeconds));
  }
}
