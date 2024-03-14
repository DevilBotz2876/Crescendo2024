package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.util.Units;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import frc.robot.subsystems.climber.ClimberIO.ClimberIOInputs;

public class ClimberIOSparkMax implements ClimberIO {
  private static final double GEAR_RATIO = 25; // make sure to change if nessessary

  // define the 2 SparkMax Controllers. A top, and a bottom
  private final CANSparkMax motor;

  private final RelativeEncoder encoder;

  public ClimberIOSparkMax(int id, boolean inverted) {
    motor = new CANSparkMax(id, MotorType.kBrushless);
    motor.restoreFactoryDefaults();

    encoder = motor.getEncoder();
    setPosition(0);

    motor.setInverted(inverted);

    motor.enableVoltageCompensation(12.0);
    motor.setSmartCurrentLimit(30);
    motor.setIdleMode(IdleMode.kBrake);

    motor.burnFlash();
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    // Set velocityRadPerSec to the encoder velocity(rotationsPerMinute) divided by the gear ratio
    // and converted into Radians Per Second
    inputs.velocityRadiansPerSecond =
        Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity() / GEAR_RATIO);
    // Get applied voltage from the top motor
    inputs.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
    inputs.current = motor.getOutputCurrent();
    inputs.positionRadians = Units.rotationsToRadians(encoder.getPosition() / GEAR_RATIO);
  }

  @Override
  public void setVoltage(double volts) {
    // Set the voltage output for the top motor
    motor.setVoltage(volts);
    // bottom.setVoltage(volts);
  }

  @Override
  public void setPosition(double position) {
    encoder.setPosition(Units.radiansToRotations(position) * GEAR_RATIO);
  }
}
