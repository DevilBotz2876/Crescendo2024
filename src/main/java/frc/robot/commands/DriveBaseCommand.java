package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.DriveBase;
import java.util.function.DoubleSupplier;

public class DriveBaseCommand extends Command {
  DriveBase drive;
  DoubleSupplier speed;
  DoubleSupplier rot;

  public DriveBaseCommand(DriveBase drive, DoubleSupplier speed, DoubleSupplier rot) {
    this.drive = drive;
    this.speed = speed;
    this.rot = rot;

    addRequirements(drive);
  }

  @Override
  public void execute() {
    ChassisSpeeds speeds =
        new ChassisSpeeds(
            speed.getAsDouble() * drive.getMaxLinearSpeed(),
            0,
            rot.getAsDouble() * drive.getMaxAngularSpeed());

    drive.runVelocity(speeds);
  }
}
