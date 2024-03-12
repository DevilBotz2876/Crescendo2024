package frc.robot.commands.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.config.RobotConfig.ArmConstants;
import frc.robot.subsystems.arm.Arm;
import java.util.function.DoubleSupplier;

public class ArmCommand extends Command {
  Arm arm;
  DoubleSupplier speed;
  double targetPosition;
  double maxArmVelocityInDegreesPerSec = ArmConstants.maxVelocityInDegreesPerSecond;

  public ArmCommand(Arm arm, DoubleSupplier speed) {
    this.arm = arm;
    this.speed = speed;

    addRequirements((Subsystem) arm);
  }

  @Override
  public void initialize() {
    targetPosition = arm.getAngle();
  }

  @Override
  public void execute() {
    double currentSpeed = speed.getAsDouble();

    targetPosition += currentSpeed * maxArmVelocityInDegreesPerSec / 50;
    targetPosition =
        MathUtil.clamp(
            targetPosition, ArmConstants.minAngleInDegrees, ArmConstants.maxAngleInDegrees);

    arm.setAngle(targetPosition);
  }
}
