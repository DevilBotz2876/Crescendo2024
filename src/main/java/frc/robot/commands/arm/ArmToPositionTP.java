// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.config.RobotConfig;
import frc.robot.subsystems.arm.Arm;
import java.util.function.DoubleSupplier;

public class ArmToPositionTP extends Command {
  private final Arm arm;
  private TrapezoidProfile motionProfile;
  private TrapezoidProfile.State initial;
  private TrapezoidProfile.State current;
  private TrapezoidProfile.State goal;
  private DoubleSupplier positionDegrees;
  private final Timer timer = new Timer();

  /** Creates a new ArmToPositionTP. */
  public ArmToPositionTP(Arm arm, DoubleSupplier positionDegrees) {
    this.arm = arm;
    this.positionDegrees = positionDegrees;

    addRequirements((Subsystem) arm);
  }

  @Override
  public void initialize() {
    motionProfile =
        new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                RobotConfig.ArmConstants.maxVelocityInDegreesPerSecond,
                RobotConfig.ArmConstants.maxAccelerationInDegreesPerSecondSquared));

    initial = new TrapezoidProfile.State(arm.getAngle(), arm.getVelocity());
    current = initial;

    goal =
        new TrapezoidProfile.State(
            MathUtil.clamp(
                positionDegrees.getAsDouble(),
                RobotConfig.ArmConstants.minAngleInDegrees,
                RobotConfig.ArmConstants.maxAngleInDegrees),
            0);
    timer.restart();
  }

  @Override
  public void execute() {
    current = motionProfile.calculate(timer.get(), initial, goal);

    // System.out.println(current.position);
    arm.setAngle(current.position, current.velocity);
  }

  public boolean isFinished() {
    return timer.hasElapsed(motionProfile.totalTime());
  }
}
