package frc.robot.commands.assist;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.arm.ArmToPositionTP;
import frc.robot.commands.shooter.SetShooterVelocity;
import frc.robot.config.RobotConfig.ArmConstants;
import frc.robot.config.RobotConfig.ShooterConstants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.shooter.Shooter;
import java.util.function.BooleanSupplier;

public class PrepareForScore extends ParallelCommandGroup {
  BooleanSupplier targetIsAmp;

  // Moves arm to ideal angle for shooting
  // Turns on shooter to ideal speed for scoring
  public PrepareForScore(Arm arm, Shooter shooter) {
    this(arm, shooter, () -> false);
  }

  public PrepareForScore(Arm arm, Shooter shooter, BooleanSupplier targetIsAmp) {
    this.targetIsAmp = targetIsAmp;
    double pos;
    double velocity;
    if (this.targetIsAmp.getAsBoolean()) {
      pos = ArmConstants.ampScoreAngleInDegrees;
      velocity = ShooterConstants.ampScoreVelocityInRPMs;
    } else {
      pos = ArmConstants.subwooferScoreAngleInDegrees;
      velocity = ShooterConstants.velocityInRPMs;
    }

    addCommands(new ArmToPositionTP(() -> pos, arm));
    addCommands(new SetShooterVelocity(shooter, () -> velocity));
  }
}
