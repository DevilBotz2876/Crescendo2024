package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.util.Units;

public class IntakeIOSparkMax implements IntakeIO {
  private static final double GEAR_RATIO = 10.0;

  // define the 1 SparkMax Controller
  private final CANSparkMax leader = new CANSparkMax(3, MotorType.kBrushless);

  // Gets the NEO encoder
  private final RelativeEncoder encoder = leader.getEncoder();

  public IntakeIOSparkMax() {
    // leader motor is not inverted, and set follower motor to follow the leader
    leader.setInverted(false);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    // Set velocityRadPerSec to the encoder velocity(rotationsPerMinute) divided by the gear ratio
    // and converted into Radians Per Second
    inputs.velocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity() / GEAR_RATIO);
    // Get applied voltage from the leader motor
    inputs.appliedVolts = leader.getAppliedOutput() * leader.getBusVoltage();

    inputs.current = leader.getOutputCurrent();
  }

  @Override
  public void setVoltage(double volts) {
    // Set the voltage output for the leader motor
    leader.setVoltage(volts);
  }
}
