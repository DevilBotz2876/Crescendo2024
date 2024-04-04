package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.arm.Arm;
import java.util.function.DoubleSupplier;

public class ArmToPosition extends Command {
  Arm arm;
  DoubleSupplier positionDegrees;
  double targetPositionDegrees;

  public ArmToPosition(Arm arm, DoubleSupplier positionDegrees) {
    this.arm = arm;
    this.positionDegrees = positionDegrees;

    addRequirements((SubsystemBase) arm);
  }

  @Override
  public void initialize() {
    targetPositionDegrees = positionDegrees.getAsDouble();

    if (Constants.debugCommands) {
      System.out.println(
          "START: " + this.getClass().getSimpleName() + " angle: " + targetPositionDegrees);
    }

    arm.setAngle(targetPositionDegrees);
  }

  @Override
  public void execute() {}

  @Override
  public boolean isFinished() {
    return arm.isAtSetpoint();
  }

  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      System.err.println("INTERRUPTED: " + this.getClass().getSimpleName());
    }

    if (Constants.debugCommands) {
      System.out.println("  END: " + this.getClass().getSimpleName());
    }
  }
}
