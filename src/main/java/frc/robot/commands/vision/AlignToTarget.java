// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.config.RobotConfig.DriveConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;
import java.util.Optional;
import java.util.function.IntSupplier;

public class AlignToTarget extends Command {
  Drive drive;
  Vision vision;
  PIDController turnPID;
  double setpoint;
  IntSupplier targetId;

  /** Creates a new AlignToTarget. */
  public AlignToTarget(Drive drive, Vision vision, IntSupplier targetId) {
    this.drive = drive;
    this.vision = vision;
    this.targetId = targetId;

    // Wild guess at P constant.
    turnPID =
        new PIDController(
            DriveConstants.anglePidKp, DriveConstants.anglePidKi, DriveConstants.anglePidKd);
    turnPID.setSetpoint(0);
    turnPID.setTolerance(DriveConstants.pidAngleErrorInDegrees);
    turnPID.enableContinuousInput(0, 360);

    setpoint = 0;

    addRequirements((Subsystem) drive);
  }

  public AlignToTarget(Drive drive, Vision vision) {
    this(drive, vision, null);

    addRequirements((Subsystem) drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Optional<Double> yawToTarget;
    if (targetId != null) {
      yawToTarget = vision.getYawToAprilTag(targetId.getAsInt());
    } else {
      yawToTarget = vision.getYawToBestTarget();
    }
    if (yawToTarget.isPresent()) {
      // Get the current heading of robot and how much we are offset from center of april tag.
      // The difference is how much we need to turn the robot to line up to center.
      //
      // TODO: check sign/math is right
      setpoint = drive.getAngle() - yawToTarget.get();
    } else {
      setpoint = drive.getAngle();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double rotate = turnPID.calculate(drive.getAngle(), setpoint);
    ChassisSpeeds speeds = new ChassisSpeeds(0, 0, rotate * drive.getMaxAngularSpeed());
    drive.runVelocity(speeds);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.runVelocity(new ChassisSpeeds());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return turnPID.atSetpoint();
  }
}
