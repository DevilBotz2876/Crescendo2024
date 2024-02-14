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
  GenericEntry velocityEntry;
  GenericEntry degreeEntry;

  public ShooterEnable(ShooterSubsystem shooter) {
    this.shooter = shooter;

    // create shooter tab on ShuffleBoard
    tab = Shuffleboard.getTab("Assist");
    // Create volt entry under Shooter tab as a number sider with min = -1 and max = 1
    velocityEntry =
        tab.add("Shooter Velocity", 0)
            .withWidget(BuiltInWidgets.kTextView)
            .withProperties(Map.of("min", 0, "max", 6000))
            .getEntry();

    tab = Shuffleboard.getTab("Assist");
    degreeEntry =
        tab.add("Shooter Angle", 60)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", 0, "max", 180))
            .getEntry();
    degreeEntry.setValue(60);
    // Sets default value to 0.0
    velocityEntry.setValue(0.0);

    addRequirements(shooter);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    // Checks the volt Entry for the volt and sets the voltage of motors
    shooter.runVelocity(velocityEntry.getDouble(0.0));
  }

  @Override
  public void end(boolean interrupted) {
    shooter.runVelocity(0);
  }
}
