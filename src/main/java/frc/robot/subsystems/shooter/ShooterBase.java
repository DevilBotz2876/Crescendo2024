package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterBase extends SubsystemBase implements Shooter {
    private final MotorController ShooterMotorLeft;
    private final MotorController ShooterMotorRight;
    private double voltage;

    public ShooterBase() {
        ShooterMotorLeft = new Spark(0);
        ShooterMotorRight = new Spark(1);
        voltage = 0;
    }

    @Override
    public void disable() {
        voltage = 0;
        ShooterMotorLeft.disable();
        ShooterMotorRight.disable();
    }

    @Override
    public void enable() {
        ShooterMotorLeft.setVoltage(voltage);
        ShooterMotorRight.setVoltage(voltage);
    }

    @Override
    //Sets the voltage to 5.0 times speed. the speed value is -1 1
    public void setSpeed(double speed) {
        voltage = 5.0 * speed;
    }
}
