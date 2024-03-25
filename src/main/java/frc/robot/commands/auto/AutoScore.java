package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.arm.ArmToPosition;
import frc.robot.commands.drive.DriveToYaw;
import frc.robot.commands.shooter.SetShooterVelocity;
import frc.robot.config.RobotConfig.ArmConstants;
import frc.robot.config.RobotConfig.DriveConstants;
import frc.robot.config.RobotConfig.ShooterConstants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import java.util.function.DoubleSupplier;

public class AutoScore extends SequentialCommandGroup {
  DoubleSupplier robotYawInDegrees;
  DoubleSupplier armAngleInDegrees;
  DoubleSupplier shooterVelocityInRPMs;

  int isFinished = 0;
  int shotPieces = 0;

  public AutoScore(
      Drive drive,
      Arm arm,
      Intake intake,
      Shooter shooter,
      DoubleSupplier robotYawInDegrees,
      DoubleSupplier armAngleInDegrees,
      DoubleSupplier shooterVelocityInRPMs) {

    if (Constants.debugCommands) {
      addCommands(
          new PrintCommand(
                  "START: AutoScore yaw: "
                      + robotYawInDegrees.getAsDouble()
                      + " angle: "
                      + armAngleInDegrees.getAsDouble()
                      + " velocity: "
                      + shooterVelocityInRPMs.getAsDouble())
              .onlyIf(() -> Constants.debugCommands));
    }
    addCommands(
        new ParallelCommandGroup(
            new DriveToYaw(drive, robotYawInDegrees)
                .withTimeout(DriveConstants.pidTimeoutInSeconds)
                .onlyIf(() -> robotYawInDegrees != null),
            new ArmToPosition(arm, armAngleInDegrees)
                .withTimeout(ArmConstants.pidTimeoutInSeconds)
                .onlyIf(() -> armAngleInDegrees != null),
            new SetShooterVelocity(shooter, shooterVelocityInRPMs)
                .withTimeout(ShooterConstants.pidTimeoutInSeconds)
                .onlyIf(() -> shooterVelocityInRPMs != null)));
    addCommands(new AutoScorePiece(intake, shooter));

    if (Constants.debugCommands) {
      addCommands(new PrintCommand("  END: AutoScore"));
    }
  }
}
