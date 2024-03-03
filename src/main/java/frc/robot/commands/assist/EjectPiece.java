package frc.robot.commands.assist;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.ArmToPositionTP;
import frc.robot.commands.intake.IntakeOut;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.intake.Intake;

public class EjectPiece extends SequentialCommandGroup {

  // Raise arm if needed
  // Run intake in reverse
  // Turn off intake

  public EjectPiece(Intake intake, Arm arm) {

    double current_pos = arm.getAngle();
    if (current_pos < 15) {
      addCommands(new ArmToPositionTP(15, arm));
    }
    addCommands(new IntakeOut(intake).withTimeout(1));
  }
}
