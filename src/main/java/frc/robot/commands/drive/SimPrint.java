package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class SimPrint extends Command {
  int isFinished = 0;
  int shotPieces = 0;

  @Override
  public void execute() {
    System.out.println("Shot a piece.");
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
