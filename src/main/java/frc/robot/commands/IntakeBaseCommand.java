package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeBase;

public class IntakeBaseCommand extends Command {
    IntakeBase intake;
    BooleanSupplier rightBumper;
    BooleanSupplier leftBumper; 
    public IntakeBaseCommand(IntakeBase intake, BooleanSupplier rightBumper, BooleanSupplier leftBumper) {
        this.intake = intake;
        this.rightBumper = rightBumper;
        this.leftBumper = leftBumper;
        
        addRequirements(intake);
    }
    @Override
    public void initialize() {
        intake.disable();
    }


    @Override
    public void execute() {
        // Turns off motors if No/All bumpers
        if ((rightBumper.getAsBoolean() && leftBumper.getAsBoolean()) || (rightBumper.getAsBoolean() == false && leftBumper.getAsBoolean() == false)) {
            // Disable the intake motors
            intake.disable();
        
        } else if (rightBumper.getAsBoolean()) { // Motors on (IN) if right bumper pressed
            intake.setSpeed(1);

            // Enable motors, It has to be called regularly for voltage compensation to work properly
            intake.enable();
        } else if (leftBumper.getAsBoolean()) { // Motors on (Out) if right bumper pressed
            intake.setSpeed(-1);

            // Enable motors, It has to be called regularly for voltage compensation to work properly
            intake.enable();
        } else { // Disable the intake motors
            intake.disable();
        }
    }
}
