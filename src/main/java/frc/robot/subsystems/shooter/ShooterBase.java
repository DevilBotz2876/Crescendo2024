package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterBase extends SubsystemBase implements Shooter {
    private final Spark ShooterMotorLeft;
    private final Spark ShooterMotorRight;
    private double voltage;

    public ShooterBase() {
        ShooterMotorLeft = new Spark(0);
        ShooterMotorRight = new Spark(1);
        voltage = 0;
    }

    @Override
    public void disable() {
        voltage = 0;
        ShooterMotorLeft.stopMotor();
        ShooterMotorRight.stopMotor();
        //System.out.println(voltage);
    }

    @Override
    public void enable() {
        ShooterMotorLeft.setVoltage(voltage);
        System.out.println(voltage);
    }

    @Override
    public void setSpeed(double speed) {
        //Sets the voltage to 5.0 times speed. the speed value is -1 1
        this.voltage = 5.0 * speed;
    }
}
