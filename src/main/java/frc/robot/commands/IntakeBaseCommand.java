package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeBase;

public class IntakeBaseCommand extends Command {
    IntakeBase intake;
    BooleanSupplier inEnable;
    BooleanSupplier outEnable; 
    public IntakeBaseCommand(IntakeBase intake, BooleanSupplier inEnable, BooleanSupplier outEnable) {
        this.intake = intake;
        this.inEnable = inEnable;
        this.outEnable = inEnable;
        
        addRequirements(intake);
    }
    @Override
    public void initialize() {
        intake.disable();
    }


    @Override
    public void execute() {
        // Turns off motors if No/All bumpers
        if (inEnable.getAsBoolean() == true && outEnable.getAsBoolean() == true) {
            // Disable the intake motors
            intake.disable();
            //System.out.println(outEnable.getAsBoolean());
        } else if(inEnable.getAsBoolean() == false && outEnable.getAsBoolean() == false) {
            intake.disable();
            //System.out.println(outEnable.getAsBoolean());
        } else if (inEnable.getAsBoolean() == true) { // Motors on (IN) if right bumper pressed
            intake.setVoltage(1);
            System.out.println(outEnable.getAsBoolean());

            // Enable motors, It has to be called regularly for voltage compensation to work properly
            intake.enable();
        } else if (outEnable.getAsBoolean()== true) { // Motors on (Out) if right bumper pressed
            intake.setVoltage(-1);
            System.out.println(outEnable.getAsBoolean());
            // Enable motors, It has to be called regularly for voltage compensation to work properly
            intake.enable();
        } else { // Disable the intake motors
            intake.disable();
        }
        //System.out.println(outEnable.getAsBoolean());
    }
}
