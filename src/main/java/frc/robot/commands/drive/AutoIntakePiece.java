package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;

public class AutoIntakePiece extends Command {
  @Override
  public void execute() {
    System.out.println("Prepare For Intake");
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
