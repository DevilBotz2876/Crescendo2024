package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class ArmSubsystem extends SubsystemBase implements Arm {
  private final ArmIO io;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
  @AutoLogOutput private double degrees;

  public ArmSubsystem(ArmIO io) {
    this.io = io;
  }

  @Override
  public double getAngle() {
    /* TODO */
    return 0;
  }

  // sets of the angle of the arm
  @Override
  public void setAngle(double degrees) {
    /* TODO: Enforce arm physical min/max limits */
    this.degrees = degrees;
  }

  @Override
  public void periodic() {
    // Updates the inputs
    io.updateInputs(inputs);
    Logger.processInputs("Arm", inputs);

    /* TODO: Implement PID control here to achieve desired angle */
  }
}
