package frc.robot.commands.assist;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

public class ScorePiece extends SequentialCommandGroup {
  // TODO: read intake voltage and read piece sensor

  public ScorePiece(Intake intake, Shooter shooter) {
    addCommands(new InstantCommand(() -> intake.setVoltage(6)));
    addCommands(new WaitCommand(3));
    addCommands(new InstantCommand(() -> intake.setVoltage(0)));
    addCommands(new InstantCommand(() -> shooter.runVelocity(0)));
  }
}
