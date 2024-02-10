package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.ArmSubsystem;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.AutoLogOutput;

public class ArmCommand extends Command {
  ArmSubsystem arm;
  BooleanSupplier moveUp;
  BooleanSupplier moveDown;
  @AutoLogOutput double targetAngle = 0;

  public ArmCommand(ArmSubsystem arm, BooleanSupplier moveUp, BooleanSupplier moveDown) {
    this.arm = arm;
    targetAngle = arm.getAngle();
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
    SmartDashboard.putNumber("ArmCommand/targetAngle", targetAngle);
    arm.setAngle(targetAngle);
  }
}
