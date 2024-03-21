package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.util.Units;

public class ShooterIOSparkMax implements ShooterIO {
  // Gear ratio for the shooter mechanism
  private static final double GEAR_RATIO = 1.0;

  // define the 1 SparkMax Controllers
  private final CANSparkMax flywheel;

  // Gets the NEO encoder
  private final RelativeEncoder encoder;

  public ShooterIOSparkMax(int id) {
    flywheel = new CANSparkMax(id, MotorType.kBrushless);
    flywheel.restoreFactoryDefaults();

    encoder = flywheel.getEncoder();
    flywheel.setInverted(false);

    flywheel.enableVoltageCompensation(12.0);
    flywheel.setSmartCurrentLimit(60, 40);
    flywheel.setSecondaryCurrentLimit(20000);

    // Set motor to brake mode so shooter stops spinning immediately
    flywheel.setIdleMode(IdleMode.kBrake);

    // Last thing we do is save all settings to flash on sparkmax
    flywheel.burnFlash();
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    // Set velocityRadPerSec to the encoder velocity(rotationsPerMinute) divided by the gear ratio
    // and converted into Radians Per Second
    inputs.velocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity() / GEAR_RATIO);
    // Get applied voltage from the top motor
    inputs.appliedVolts = flywheel.getAppliedOutput() * flywheel.getBusVoltage();
    // Get applied voltage from the top motor
    inputs.current = flywheel.getOutputCurrent();
  }

  public boolean supportsHardwarePid() {
    return false;
  }

  @Override
  public void setVoltage(double volts) {
    // Set the voltage output for the top motor
    flywheel.setVoltage(volts);
  }
}
