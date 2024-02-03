package frc.robot.subsystems.arm;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class ArmSubsystem extends SubsystemBase implements Arm {
  private final ArmIO io;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
  @AutoLogOutput private double degrees = 0;
  ArmFeedforward feedforward;

  public ArmSubsystem(ArmIO io) {
    this.io = io;

    // TODO: These are sample values.  Need to run sysid on shooter and get real values.
    feedforward = new ArmFeedforward(1, 1, 1);
  }

  @Override
  public double getAngle() {
    /* TODO */
    return Units.radiansToDegrees(inputs.positionRad);
    // return 0;
  }

  // sets of the angle of the arm
  @Override
  public void setAngle(double degrees) {
    /* TODO: Enforce arm physical min/max limits */
    this.degrees = degrees;

    // TODO: The actual feedforward voltage needs to be computed!
    io.setPosition(
        Units.degreesToRadians(degrees), feedforward.calculate(Units.degreesToRadians(degrees), 1));
  }

  @Override
  public void periodic() {
    // Updates the inputs
    io.updateInputs(inputs);
    Logger.processInputs("Arm", inputs);

    /* TODO: Implement PID control here to achieve desired angle */
  }
}
