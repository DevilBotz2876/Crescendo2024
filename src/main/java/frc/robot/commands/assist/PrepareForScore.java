package frc.robot.commands.assist;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.arm.ArmToPositionTP;
import frc.robot.commands.shooter.SetShooterVelocity;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.shooter.Shooter;
import java.util.function.DoubleSupplier;

public class PrepareForScore extends ParallelCommandGroup {
  public PrepareForScore(
      Arm arm, Shooter shooter, DoubleSupplier armAngle, DoubleSupplier shooterVelocity) {
    addCommands(new ArmToPositionTP(armAngle, arm));
    addCommands(new SetShooterVelocity(shooter, shooterVelocity));
  }
}
