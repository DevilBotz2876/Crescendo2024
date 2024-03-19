package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.IntakeIO;

public class LedSystem extends SubsystemBase implements Led {
    // private final LedIO io;
    private final LedIOInputsAutoLogged inputs = new LedIOInputsAutoLogged();
    public void setColor(int red, int blue, int green) {}
    public int getRed() {return inputs.red;}
    public int getGreen() {return inputs.green;}
    public int getBlue() {return inputs.blue;}
}

