package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.drive.Drive;
import java.util.function.DoubleSupplier;

public class DriveToYaw extends Command {
  Drive drive;
  DoubleSupplier yawDegrees;
  double targetYaw;
  PIDController turnPID = new PIDController(0.1, 0, 0);

  public DriveToYaw(Drive drive, DoubleSupplier yawDegrees) {
    this.drive = drive;
    this.yawDegrees = yawDegrees;

    turnPID.setTolerance(0.5);
    turnPID.enableContinuousInput(0, 360);
    addRequirements((Subsystem) drive);
  }

  @Override
  public void initialize() {
    targetYaw = this.yawDegrees.getAsDouble();
    turnPID.reset();
    turnPID.setSetpoint(targetYaw);
    System.out.println("START: " + this.getClass().getSimpleName() + " yaw: " + targetYaw);
  }

  @Override
  public void execute() {
    double rotate = turnPID.calculate(drive.getAngle());
    ChassisSpeeds speeds = new ChassisSpeeds(0, 0, rotate * drive.getMaxAngularSpeed());
    drive.runVelocity(speeds);
  }

  @Override
  public boolean isFinished() {
    return turnPID.atSetpoint();
  }

  @Override
  public void end(boolean interrupted) {
    drive.runVelocity(new ChassisSpeeds());
    System.out.println("  END: " + this.getClass().getSimpleName());
  }
}
