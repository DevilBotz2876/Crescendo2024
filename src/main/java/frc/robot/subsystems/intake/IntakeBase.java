package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeBase extends SubsystemBase implements Intake {
    private final MotorController IntakeMotor;

    private double voltage;

    public IntakeBase() {
        IntakeMotor = new Spark(0);
        voltage = 0;
    }

    @Override
    public void disable() {
        voltage = 0;
        IntakeMotor.setVoltage(voltage);
    }

    @Override
    public void enable() {
        IntakeMotor.setVoltage(voltage);
    }

    @Override
    //Sets the voltage to 5.0 times speed. the speed value is -1 1
    public void setSpeed(double speed) {
        voltage = 5.0 * speed;
    }
}
