package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.ArmSubsystem;

/**
 * This class uses a widget on Shuffleboard to control the arm setpoint. It is meant to be used for
 * debug/test/bring type work.
 */
public class ArmToPosition extends Command {
  ArmSubsystem arm;
  double positionDegrees;

  public ArmToPosition(ArmSubsystem arm, double positionDegrees) {
    this.arm = arm;
    this.positionDegrees = positionDegrees;

    addRequirements(arm);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    arm.setAngle(positionDegrees);
  }

  @Override
  public void end(boolean interrupted) {
    arm.runVoltage(0);
  }
}
