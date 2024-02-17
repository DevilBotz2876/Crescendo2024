package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.RobotConfig.ArmConstants;
import frc.robot.subsystems.arm.Arm;
import java.util.function.DoubleSupplier;

/**
 * This class uses a widget on Shuffleboard to control the arm setpoint. It is meant to be used for
 * debug/test/bring type work.
 */
public class ArmToPosition extends Command {
  Arm arm;
  DoubleSupplier positionDegrees;
  double timeMS;

  public ArmToPosition(Arm arm, DoubleSupplier positionDegrees) {
    this.arm = arm;
    this.positionDegrees = positionDegrees;

    addRequirements((SubsystemBase) arm);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    arm.setAngle(positionDegrees.getAsDouble());
  }

  @Override
  public boolean isFinished() {

    if (arm.getAngle() > positionDegrees.getAsDouble() - ArmConstants.pidAngleErrorInDegrees
        && arm.getAngle() < positionDegrees.getAsDouble() + ArmConstants.pidAngleErrorInDegrees) {
      timeMS += 20.0;
      if (timeMS == 1000) {
        SmartDashboard.putBoolean("Arm/ArmToPosition/isFinished", true);
        return true;
      }
    } else {
      timeMS = 0.0;
    }
    SmartDashboard.putBoolean("Arm/ArmToPosition/isFinished", false);
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    // arm.runVoltage(0);
  }
}
