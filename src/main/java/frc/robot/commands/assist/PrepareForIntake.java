package frc.robot.commands.assist;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.intake.Intake;

public class PrepareForIntake extends SequentialCommandGroup {
  // TODO:  read arm angle and intake voltage from GUI

  public PrepareForIntake(Arm arm, Intake intake) {
    addCommands(new InstantCommand(() -> arm.setAngle(1)));
    addCommands(new InstantCommand(() -> intake.setVoltage(6)));
  }
}
