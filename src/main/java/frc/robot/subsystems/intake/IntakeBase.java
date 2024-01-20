package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

//import frc.robot.subsystems.intake.IntakeIO;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeBase extends SubsystemBase implements Intake {
    private final IntakeIO IO;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
    private double voltage;

    public IntakeBase(IntakeIO IO) {
        
        this.IO = IO; 
            
        voltage = 0;
    }

    @Override
    //disable the intake
    public void disable() {
        voltage = 0;
        IO.setVoltage(voltage);
    }

    @Override
    //Enabling the intake
    public void enable() {
        //voltage = 1;
        IO.setVoltage(voltage);
    }

    @Override
    public void setVoltage(double volts) {
        voltage = volts;
    }
    @Override
    public void periodic() {
        IO.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);
    }
}
