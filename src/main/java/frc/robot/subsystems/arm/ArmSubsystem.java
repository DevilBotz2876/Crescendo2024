package frc.robot.subsystems.arm;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class ArmSubsystem extends SubsystemBase implements Arm {
  private final ArmIO io;
  private final ArmFeedforward Armff;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
  @AutoLogOutput private double degrees;
  @AutoLogOutput private double positionRad;

  public ArmSubsystem(ArmIO io) {
    this.io = io;
    // TODO: These are sample values.
    Armff = new ArmFeedforward(0.1, 1, 0.05, 0);
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
    final double minAngle = 0;
    final double maxAngle = 180;

    // Check if the angle is below the minimum limit or above the maximum limit
    // If it is the it is set to min/max
    if (degrees < minAngle) {
      this.degrees = minAngle; // Set to the minimum angle
    } else if (degrees > maxAngle) {
      this.degrees = maxAngle; // Set to the maximum angle
    }
    // The  angle is within the range and is set
    else {
      this.degrees = degrees;
    }

    // target
    double TargetRad = Units.degreesToRadians(degrees);

    // Calculate feedforward voltage with ArmFeedforward
    double ffVolts = Armff.calculate(TargetRad, 1);

    // Set the position reference with feedforward voltage
    io.setPosition(TargetRad, ffVolts);
  }

  @Override
  public void periodic() {
    // Updates the inputs
    io.updateInputs(inputs);
    Logger.processInputs("Arm", inputs);

    /* TODO: Implement PID control here to achieve desired angle */
  }
}
