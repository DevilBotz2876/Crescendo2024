package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.ArmSubsystem;
import java.util.function.BooleanSupplier;

public class ArmCommand extends Command {
  ArmSubsystem arm;
  BooleanSupplier moveUp;
  BooleanSupplier moveDown;
  double targetAngle = 0;

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
      System.out.println("Arm Up");
      targetAngle++;
    } else if (moveDown.getAsBoolean()) {
      System.out.println("Arm Down");
      targetAngle--;
    }
    arm.setAngle(targetAngle);
  }
}
