package frc.robot.commands.drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.DriveBase;
import java.util.function.DoubleSupplier;

public class DriveCommand extends Command {
  DriveBase drive;
  DoubleSupplier speedX;
  DoubleSupplier speedY;
  DoubleSupplier rot;

  public DriveCommand(
      DriveBase drive, DoubleSupplier speedX, DoubleSupplier speedY, DoubleSupplier rot) {
    this.drive = drive;
    this.speedX = speedX;
    this.speedY = speedY;
    this.rot = rot;

    addRequirements(drive);
  }

  @Override
  public void execute() {
    ChassisSpeeds speeds =
        new ChassisSpeeds(
            speedX.getAsDouble() * drive.getMaxLinearSpeed(),
            speedY.getAsDouble() * drive.getMaxLinearSpeed(),
            rot.getAsDouble() * drive.getMaxAngularSpeed());

    drive.runVelocity(speeds);
  }
}
