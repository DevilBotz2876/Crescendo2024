package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.config.RobotConfig.DriveConstants;
import frc.robot.subsystems.drive.Drive;
import java.util.function.DoubleSupplier;

public class DriveTranslateInX extends Command {
  Drive drive;
  DoubleSupplier xMeters;
  double xTargetMeters;
  PIDController xTranslatePID =
      new PIDController(
          DriveConstants.translatePidKp, DriveConstants.translatePidKi, DriveConstants.translatePidKd);
  Timer timer = new Timer();

  public DriveTranslateInX(Drive drive, DoubleSupplier xMeters) {
    this.drive = drive;
    this.xMeters = xMeters;

    xTranslatePID.setTolerance(DriveConstants.translatePidErrorInMeters);
    addRequirements((Subsystem) drive);
  }

  @Override
  public void initialize() {
    xTargetMeters = drive.getPose().getX() + xMeters.getAsDouble();

    SmartDashboard.putNumber("DriveTranslateInX/xMeters", xMeters.getAsDouble());
    SmartDashboard.putNumber("DriveTranslateInX/xTargetMeters", xTargetMeters);
    xTranslatePID.reset();
    xTranslatePID.setSetpoint(xTargetMeters);

    timer.reset();
    if (Constants.debugCommands) {
      System.out.println(
          "START: "
              + this.getClass().getSimpleName()
              + " X: "
              + xTargetMeters
              + " currentX: "
              + drive.getPose().getX());
    }
  }

  @Override
  public void execute() {
    double translate = xTranslatePID.calculate(drive.getPose().getX());
    ChassisSpeeds speeds = new ChassisSpeeds(translate, 0, 0);
    SmartDashboard.putNumber("DriveTranslateInX/speeds", speeds.vxMetersPerSecond);
    // drive.runVelocity(speeds);
  }

  @Override
  public boolean isFinished() {
    if (xTranslatePID.atSetpoint()) {
        return true;
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
