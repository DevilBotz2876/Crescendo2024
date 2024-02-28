package frc.robot.commands.assist;

import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.DriveToYaw;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import java.util.function.DoubleSupplier;

public class AutoOrientAndShoot extends SequentialCommandGroup {
  DoubleSupplier robotYawInDegrees;

  public AutoOrientAndShoot(
      Drive drive, Intake intake, Shooter shooter, DoubleSupplier robotYawInDegrees) {
    super(
        new PrintCommand(
            "START: AutoOrientAndShoot" + " yaw: " + robotYawInDegrees.getAsDouble() + " angle: "),
        new DriveToYaw(drive, robotYawInDegrees),
        new ScorePiece(intake, shooter),
        new PrintCommand("  END: AutoOrientAndShoot"));
  }
}
