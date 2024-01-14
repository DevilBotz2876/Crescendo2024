package frc.robot.commands;



import java.util.Map;
import java.util.function.DoubleSupplier;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterBase;

public class ShooterBaseCommand extends Command {
    ShooterBase shooter;
    DoubleSupplier rightTriggerAxis;
    ShuffleboardTab tab;
    GenericEntry speedEntry;

    public ShooterBaseCommand(ShooterBase shooter, DoubleSupplier rightTriggerAxis) {
        this.shooter = shooter;
        this.rightTriggerAxis = rightTriggerAxis;

        //create shooter tab on ShuffleBoard
        tab = Shuffleboard.getTab("Shooter");

        addRequirements(shooter);
    }
    @Override
    public void initialize() {
        // create speed entry under Shooter tab as a number sider with min = -1 and max = 1
        speedEntry = tab.add("speed", 0).withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", -1, "max", 1)).getEntry();

        //sets default value to 0.0
        speedEntry.setValue(0.0);
    }


    @Override
    public void execute() {
        //turns on motors if right trigger is fully pressed. else the motors turn off.
        if (rightTriggerAxis.getAsDouble() == 1.0) {
            shooter.setSpeed(speedEntry.getDouble(0.0));
            shooter.enable();
        } else {
            shooter.disable();
        }
    }
}
