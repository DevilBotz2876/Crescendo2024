package frc.robot.commands;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class ShooterEnable extends Command {
  ShooterSubsystem shooter;
  NetworkTable assistGUI = NetworkTableInstance.getDefault().getTable("Shuffleboard/Assist");

  GenericEntry velocityEntry;

  public ShooterEnable(ShooterSubsystem shooter) {
    this.shooter = shooter;

    addRequirements(shooter);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    // Checks the volt Entry for the volt and sets the voltage of motors
    shooter.runVelocity(assistGUI.getEntry("Shooter Velocity").getDouble(3000));
  }

  @Override
  public void end(boolean interrupted) {
    shooter.runVelocity(0);
  }
}
