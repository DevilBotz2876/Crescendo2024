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
  @AutoLogOutput private double angleIncrement = 6;
  @AutoLogOutput private double targetAngle = 0;

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
    targetAngle = arm.getAngle();

    if (moveUp.getAsBoolean()) {
      // targetAngle = arm.getAngle();
      System.out.println("Arm Up");
      targetAngle += angleIncrement;
    } else if (moveDown.getAsBoolean()) {
      // targetAngle = arm.getAngle();
      // System.out.println("Arm Down");
      targetAngle -= angleIncrement;
    }
    if (targetAngle < 0) {
      targetAngle = 0;
    } else if (targetAngle > 90) {
      targetAngle = 90;
    }
    SmartDashboard.putNumber("ArmCommand/targetAngle", targetAngle);
    arm.setAngle(targetAngle);
  }
}
