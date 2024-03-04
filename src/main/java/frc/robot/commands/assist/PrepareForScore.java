package frc.robot.commands.assist;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.arm.ArmToPositionTP;
import frc.robot.commands.shooter.SetShooterVelocity;
import frc.robot.config.RobotConfig.ArmConstants;
import frc.robot.config.RobotConfig.ShooterConstants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.shooter.Shooter;
import java.util.function.BooleanSupplier;

public class PrepareForScore extends ParallelCommandGroup {
  // TODO:  read arm angle and Shooter: Velocity from GUI
  NetworkTable assistGUI = NetworkTableInstance.getDefault().getTable("Shuffleboard/Assist");
  BooleanSupplier targetIsAmp;

  // Moves arm to ideal angle for shooting
  // Turns on shooter to ideal speed for scoring
  public PrepareForScore(Arm arm, Shooter shooter) {
    this(arm, shooter, () -> false);
  }

  public PrepareForScore(Arm arm, Shooter shooter, BooleanSupplier targetIsAmp) {
    this.targetIsAmp = targetIsAmp;
    double pos;
    if (this.targetIsAmp.getAsBoolean()) {
      pos = ArmConstants.ampScoreShooterAngleInDegrees;
    } else {
      pos = ArmConstants.subwooferScoreAngleInDegrees;
    }

    addCommands(
        // new ArmToPosition(
        //     (ArmSubsystem) arm,
        //     () ->
        //         this.targetIsAmp.getAsBoolean()
        //             ? assistGUI
        //                 .getEntry("Shooter: Angle (Amp)")
        //                 .getDouble(ArmConstants.ampScoreShooterAngleInDegrees)
        //             : assistGUI
        //                 .getEntry("Shooter: Angle")
        //                 .getDouble(ArmConstants.shooterAngleInDegrees)));
        new ArmToPositionTP(() -> pos, arm));
    addCommands(
        new SetShooterVelocity(
            shooter,
            () ->
                this.targetIsAmp.getAsBoolean()
                    ? assistGUI
                        .getEntry("Shooter: Velocity (Amp)")
                        .getDouble(ShooterConstants.ampScoreVelocityInRPMs)
                    : assistGUI
                        .getEntry("Shooter: Velocity")
                        .getDouble(ShooterConstants.velocityInRPMs)));
  }
}
