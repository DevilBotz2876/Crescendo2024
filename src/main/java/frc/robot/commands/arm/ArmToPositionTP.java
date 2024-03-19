// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;
import frc.robot.config.RobotConfig;
import frc.robot.subsystems.arm.Arm;
import java.util.function.DoubleSupplier;

public class ArmToPositionTP extends TrapezoidProfileCommand {
  /** Creates a new ArmToPositionTP. */
  public ArmToPositionTP(Arm arm, DoubleSupplier positionDegrees) {
    super(
        // The motion profile to be executed
        new TrapezoidProfile(
            // The motion profile constraints
            new TrapezoidProfile.Constraints(
                RobotConfig.ArmConstants.maxVelocityInDegreesPerSecond,
                RobotConfig.ArmConstants.maxAccelerationInDegreesPerSecondSquared)),
        state -> {
          // Apply current trajectory state to the arm.  We are setting the angle/setpoint
          // calculated by the trapezoid profile
          arm.setAngle(state.position, state.velocity);

          // Should be logged in subsystem, delete this after verifying values are logged.
          // Logger.recordOutput("Arm/TP/curPos", state.position);
          // Logger.recordOutput("Arm/TP/curVel", state.velocity);
        },
        // Goal state, we want to get to requested position and hold arm there, so velocity is zero.
        () ->
            new TrapezoidProfile.State(
                MathUtil.clamp(
                    positionDegrees.getAsDouble(),
                    RobotConfig.ArmConstants.minAngleInDegrees,
                    RobotConfig.ArmConstants.maxAngleInDegrees),
                0),
        // Current state
        () -> {
          System.out.println(
              "current angle/velocity = " + arm.getAngle() + "/" + arm.getVelocity());
          return new TrapezoidProfile.State(arm.getAngle(), arm.getVelocity());
        },
        arm);
  }
}
