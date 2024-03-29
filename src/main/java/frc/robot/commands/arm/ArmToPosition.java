package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.config.RobotConfig.ArmConstants;
import frc.robot.subsystems.arm.Arm;
import java.util.function.DoubleSupplier;

public class ArmToPosition extends Command {
  Arm arm;
  DoubleSupplier positionDegrees;
  double targetPositionDegrees;
  Timer timer = new Timer();

  public ArmToPosition(Arm arm, DoubleSupplier positionDegrees) {
    this.arm = arm;
    this.positionDegrees = positionDegrees;

    addRequirements((SubsystemBase) arm);
  }

  @Override
  public void initialize() {
    targetPositionDegrees = positionDegrees.getAsDouble();
    timer.restart();

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
    if (Math.abs(arm.getAngle() - targetPositionDegrees) <= ArmConstants.pidAngleErrorInDegrees) {
      if (timer.get() >= ArmConstants.pidSettlingTimeInSeconds) {
        return true;
      }
    } else {
      timer.reset();
    }
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    // arm.runVoltage(0);
    if (interrupted) {
      System.err.println("INTERRUPTED: " + this.getClass().getSimpleName());
    }

    if (Constants.debugCommands) {
      System.out.println("  END: " + this.getClass().getSimpleName());
    }
  }
}
