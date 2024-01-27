package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.util.Units;

public class ShooterIOSparkMax implements ShooterIO {
  // Gear ratio for the shooter mechanism
  private static final double GEAR_RATIO = 1.0;

  // define the 2 SparkMax Controllers. A top, and a bottom
  private final CANSparkMax top = new CANSparkMax(2, MotorType.kBrushless);
  private final CANSparkMax bottom = new CANSparkMax(1, MotorType.kBrushless);

  // Gets the NEO encoder
  private final RelativeEncoder topEncoder = top.getEncoder();
  private final RelativeEncoder bottomEncoder = top.getEncoder();

  public ShooterIOSparkMax() {
    // top motor is not inverted, and set bottom motor to follow the top
    top.setInverted(false);
    bottom.setInverted(false);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    // Set velocityRadPerSec to the encoder velocity(rotationsPerMinute) divided by the gear ratio
    // and converted into Radians Per Second
    inputs.velocityRadPerSecTop =
        Units.rotationsPerMinuteToRadiansPerSecond(topEncoder.getVelocity() / GEAR_RATIO);
    // Get applied voltage from the top motor
    inputs.appliedVoltsTop = top.getAppliedOutput() * top.getBusVoltage();

    inputs.velocityRadPerSecBottom =
        Units.rotationsPerMinuteToRadiansPerSecond(bottomEncoder.getVelocity() / GEAR_RATIO);
    // Get applied voltage from the top motor
    inputs.appliedVoltsBottom = bottom.getAppliedOutput() * bottom.getBusVoltage();
  }

  @Override
  public void setVoltage(double volts) {
    // Set the voltage output for the top motor
    top.setVoltage(volts);
    bottom.setVoltage(volts);
  }
}
