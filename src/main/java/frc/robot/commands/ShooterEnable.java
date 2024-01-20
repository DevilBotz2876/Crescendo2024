package frc.robot.commands;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import java.util.Map;

public class ShooterEnable extends Command {
  ShooterSubsystem shooter;
  ShuffleboardTab tab;
  GenericEntry voltsEntry;

  public ShooterEnable(ShooterSubsystem shooter) {
    this.shooter = shooter;

    // create shooter tab on ShuffleBoard
    tab = Shuffleboard.getTab("Shooter");
    // Create volt entry under Shooter tab as a number sider with min = -1 and max = 1
    voltsEntry =
        tab.add("Volts", 0)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", -12, "max", 12))
            .getEntry();

    // Sets default value to 0.0
    voltsEntry.setValue(0.0);

    addRequirements(shooter);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    // Checks the volt Entry for the volt and sets the voltage of motors
    shooter.setVoltage(voltsEntry.getDouble(0.0));

    // Enable motors, It has to be called regularly for voltage compensation to work properly
    shooter.enable();
  }

  @Override
  public void end(boolean interrupted) {}
}
