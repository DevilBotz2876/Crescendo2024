package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.ArmSubsystem;
import java.util.function.BooleanSupplier;

public class ArmCommand extends Command {
  ArmSubsystem arm;
  BooleanSupplier moveUp;
  BooleanSupplier moveDown;

  public ArmCommand(ArmSubsystem arm, BooleanSupplier moveUp, BooleanSupplier moveDown) {
    this.arm = arm;
    this.moveUp = moveUp;
    this.moveDown = moveDown;

    addRequirements(arm);
  }

  @Override
  public void execute() {
    /* TODO: Implement arm controls here to increase/decrease desired angle */
    if (moveUp.getAsBoolean()) {
      // Increase the angle
      double newAngle = arm.getAngle() + 1.0;
      arm.setAngle(newAngle);
    } else if (moveDown.getAsBoolean()) {
      // Decrease the angle
      double newAngle = arm.getAngle() - 1.0;
      arm.setAngle(newAngle);
    }
  }
}
