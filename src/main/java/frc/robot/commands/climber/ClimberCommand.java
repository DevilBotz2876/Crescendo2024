package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.climber.Climber;

public class ClimberCommand extends Command {
  private Climber climber;
  private boolean bExtend;

  public ClimberCommand(Climber climber, boolean bExtend) {
    this.climber = climber;
    this.bExtend = bExtend;

    addRequirements((Subsystem) climber);
  }

  @Override
  public void initialize() {
    if (bExtend) {
      climber.extend();
    } else {
      climber.retract();
    }
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
