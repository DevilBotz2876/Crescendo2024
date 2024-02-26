package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import java.util.function.DoubleSupplier;

public class DriveToYaw extends Command {
  Drive drive;
  DoubleSupplier yawDegrees;

  public DriveToYaw(Drive drive, DoubleSupplier yawDegrees) {
    this.drive = drive;
    this.yawDegrees = yawDegrees;
  }

  @Override
  public void execute() {
    /* TODO: Turn to specified yaw */
  }

  @Override
  public boolean isFinished() {
    /* TODO: done when we've reached the desired yaw */
    return true;
  }
}
