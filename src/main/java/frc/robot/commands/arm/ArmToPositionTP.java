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
  private TrapezoidProfile.State startState;
  private TrapezoidProfile.State targetState;
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

    startState = new TrapezoidProfile.State(arm.getAngle(), arm.getVelocity());

    targetState =
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
    TrapezoidProfile.State setPoint = motionProfile.calculate(timer.get(), startState, targetState);

    //    System.out.println(("position/velocity=" + timer.get() + "/" + motionProfile.totalTime() +
    // ":" + setPoint.position + "/" + setPoint.velocity));
    arm.setAngle(setPoint.position, setPoint.velocity);
  }

  public boolean isFinished() {
    return timer.hasElapsed(motionProfile.totalTime());
  }
}
