package frc.robot.subsystems.shooter;


import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterBase extends SubsystemBase implements Shooter {
    ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
    private double voltage;

    public ShooterBase(ShooterIO io) {
        this.io = io;
        voltage = 0;
    }

    @Override
    //Disable the shooter
    public void disable() {
        voltage = 0;
        io.setVoltage(voltage);
    }

    @Override
    //Enable the shooter
    public void enable() {
        io.setVoltage(voltage);
    }

    @Override
    //Sets the voltage to volts. the volts value is -12 to 12
    public void setVoltage(double volts) {
        voltage = volts;
    }

    @Override
    public double getCurrentSpeed() {
        return inputs.velocityRadPerSec;
    }
    
    @Override
    public void periodic() {
        //Updates the inputs
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);
    }
}
