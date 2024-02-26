package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.ArmToPosition;
import frc.robot.commands.assist.ScorePiece;
import frc.robot.commands.drive.DriveToYaw;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

public class AutoShootPiece extends SequentialCommandGroup {
  double robotYawInDegrees;
  double armAngleInDegrees;
  double shooterVelocityInRPMs;

  int isFinished = 0;
  int shotPieces = 0;

  public AutoShootPiece(
      Drive drive,
      Arm arm,
      Shooter shooter,
      Intake intake,
      double robotYawInDegrees,
      double armAngleInDegrees,
      double shooterVelocityInRPMs) {
    super(
        new ParallelCommandGroup(
            new DriveToYaw(drive, () -> robotYawInDegrees),
            new ArmToPosition(arm, () -> armAngleInDegrees)),
        new ScorePiece(intake, shooter));
  }
}
