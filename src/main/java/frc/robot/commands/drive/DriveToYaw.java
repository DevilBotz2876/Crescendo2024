package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.config.RobotConfig.DriveConstants;
import frc.robot.subsystems.drive.Drive;
import java.util.function.DoubleSupplier;

public class DriveToYaw extends Command {
  Drive drive;
  DoubleSupplier yawDegrees;
  double targetYaw;
  PIDController turnPID =
      new PIDController(
          DriveConstants.rotatePidKp, DriveConstants.rotatePidKi, DriveConstants.rotatePidKd);
  Timer timer = new Timer();

  public DriveToYaw(Drive drive, DoubleSupplier yawDegrees) {
    this.drive = drive;
    this.yawDegrees = yawDegrees;

    turnPID.setTolerance(DriveConstants.rotatePidErrorInDegrees);
    turnPID.enableContinuousInput(-180, 180);
    addRequirements((Subsystem) drive);
  }

  @Override
  public void initialize() {
    targetYaw = this.yawDegrees.getAsDouble();
    turnPID.reset();
    turnPID.setSetpoint(targetYaw);
    timer.reset();
    if (Constants.debugCommands) {
      System.out.println(
          "START: "
              + this.getClass().getSimpleName()
              + " yaw: "
              + targetYaw
              + " currentYaw: "
              + drive.getAngle());
    }
  }

  @Override
  public void execute() {
    double rotate = turnPID.calculate(drive.getAngle());
    ChassisSpeeds speeds = new ChassisSpeeds(0, 0, rotate * drive.getMaxAngularSpeed());
    drive.runVelocity(speeds);
  }

  @Override
  public boolean isFinished() {
    if (turnPID.atSetpoint()) {
      if (timer.get() >= DriveConstants.pidSettlingTimeInSeconds) {
        return true;
      }
    } else {
      timer.reset();
    }
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      System.err.println("INTERRUPTED: " + this.getClass().getSimpleName());
    }

    drive.runVelocity(new ChassisSpeeds());
    if (Constants.debugCommands) {
      System.out.println("  END: " + this.getClass().getSimpleName());
    }
  }
}
