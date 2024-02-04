// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.DriveBase;
import frc.robot.subsystems.vision.Vision;

public class AlignToTarget extends Command {
  DriveBase drive;
  Vision vision;
  PIDController turnPID;

  /** Creates a new AlignToTarget. */
  public AlignToTarget(DriveBase drive, Vision vision) {
    this.drive = drive;
    this.vision = vision;

    // Wild guess at P constant.
    turnPID = new PIDController(.1, 0, 0);
    turnPID.setSetpoint(0);
    turnPID.setTolerance(2);

    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (vision.hasTarget()) {
      double rotate = turnPID.calculate(vision.getYaw(), 0);
      ChassisSpeeds speeds = new ChassisSpeeds(0, 0, rotate * drive.getMaxAngularSpeed());
      drive.runVelocity(speeds);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return turnPID.atSetpoint();
  }
}
