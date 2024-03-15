// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.config.RobotConfig.ArmConstants;
import frc.robot.subsystems.arm.Arm;

public class ArmToPositionPP extends ProfiledPIDCommand {

  /** Creates a new ArmToPositionTP. */
  public ArmToPositionPP(DoubleSupplier positionDegrees, Arm arm) {

    super(
        new ProfiledPIDController(
          ArmConstants.pidKp, ArmConstants.pidKi, ArmConstants.pidKd, new TrapezoidProfile.Constraints(
                ArmConstants.maxVelocity, ArmConstants.maxAcceleration)),
    () -> arm.getAngle(),
    positionDegrees,
    (output, setpoint) -> arm.runVoltage(output),
    arm
    );
    
  }

  public boolean isFinished() {
    boolean done = super.isFinished();
    Logger.recordOutput("Arm/PP/isFinished", done);
    return done;
  }
}
