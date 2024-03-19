package frc.robot.commands.assist;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.arm.ArmToPosition;
import frc.robot.commands.intake.IntakeOut;
import frc.robot.config.RobotConfig;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

public class EjectPiece extends SequentialCommandGroup {

  // Raise arm if needed
  // Run intake in reverse
  // Turn off intake

  public EjectPiece(Intake intake, Arm arm, Shooter shooter) {

    double current_pos = arm.getAngle();
    if (current_pos < RobotConfig.ArmConstants.ejectAngleInDegrees) {
      addCommands(new ArmToPosition(arm, () -> RobotConfig.ArmConstants.ejectAngleInDegrees));
    }

    addCommands(
        new ParallelCommandGroup(
                new SequentialCommandGroup(
                    new InstantCommand(() -> shooter.runVoltage(-12.0)),
                    new WaitCommand(0.50),
                    new InstantCommand(() -> shooter.runVoltage(0.0))),
                new IntakeOut(intake))
            .withTimeout(1));
  }
}
