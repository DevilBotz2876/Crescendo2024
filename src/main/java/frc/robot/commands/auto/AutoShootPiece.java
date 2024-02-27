package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.ArmToPosition;
import frc.robot.commands.assist.ScorePiece;
import frc.robot.commands.drive.DriveToYaw;
import frc.robot.commands.shooter.SetShooterVelocity;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import java.util.function.DoubleSupplier;

public class AutoShootPiece extends SequentialCommandGroup {
  DoubleSupplier robotYawInDegrees;
  DoubleSupplier armAngleInDegrees;
  DoubleSupplier shooterVelocityInRPMs;

  int isFinished = 0;
  int shotPieces = 0;

  public AutoShootPiece(
      Drive drive,
      Arm arm,
      Shooter shooter,
      Intake intake,
      DoubleSupplier robotYawInDegrees,
      DoubleSupplier armAngleInDegrees,
      DoubleSupplier shooterVelocityInRPMs) {
    super(
        new PrintCommand(
            "START: AutoShootPiece yaw: "
                + robotYawInDegrees
                + " angle: "
                + armAngleInDegrees
                + " velocity: "
                + shooterVelocityInRPMs),
        new ParallelCommandGroup(
            new DriveToYaw(drive, robotYawInDegrees),
            new ArmToPosition(arm, armAngleInDegrees),
            new SetShooterVelocity(shooter, shooterVelocityInRPMs)),
        new ScorePiece(intake, shooter),
        new PrintCommand("  END: AutoShootPiece"));
  }
}
