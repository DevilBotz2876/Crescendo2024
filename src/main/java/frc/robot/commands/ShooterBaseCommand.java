package frc.robot.commands;



import java.util.Map;
import java.util.function.BooleanSupplier;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterBase;

public class ShooterBaseCommand extends Command {
    ShooterBase shooter;
    BooleanSupplier enable;
    ShuffleboardTab tab;
    GenericEntry voltsEntry;

    public ShooterBaseCommand(ShooterBase shooter, BooleanSupplier enable) {
        this.shooter = shooter;
        this.enable = enable;

        //create shooter tab on ShuffleBoard
        tab = Shuffleboard.getTab("Shooter");
        // Create volt entry under Shooter tab as a number sider with min = -1 and max = 1
        voltsEntry = tab.add("Volts", 0).withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", -12, "max", 12)).getEntry();

        // Sets default value to 0.0
        voltsEntry.setValue(0.0);

        addRequirements(shooter);
    }
    @Override
    public void initialize() {
        // disable shooter when initialize
        shooter.disable();
    }


    @Override
    public void execute() {
        // Turns on motors if right trigger is fully pressed. else the motors turn off.
        if (enable.getAsBoolean() == true) {
            // Checks the volt Entry for the volt and sets the voltage of motors
            shooter.setVoltage(voltsEntry.getDouble(0.0));

            // Enable motors, It has to be called regularly for voltage compensation to work properly
            shooter.enable();
        } else {
            // Disable the shooters
            shooter.disable();
        }
    }

    @Override
    //disable shooter when it ends.
    public void end(boolean interrupted) {
        shooter.disable();
    }
}
