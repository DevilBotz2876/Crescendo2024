package frc.robot.commands.assist;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.shooter.Shooter;

public class PrepareForScore extends SequentialCommandGroup {
  // TODO:  read arm angle and shooter velocity from GUI

  public PrepareForScore(Arm arm, Shooter shooter) {
    addCommands(new InstantCommand(() -> arm.setAngle(60)));
    addCommands(new InstantCommand(() -> shooter.runVelocity(3000)));
  }
}
