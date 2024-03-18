package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;

public class IntakeIOSparkMax implements IntakeIO {
  private static final double GEAR_RATIO = 4.0;

  // define the 1 SparkMax Controller
  private final CANSparkMax leader;
  // Gets the NEO encoder
  private final RelativeEncoder encoder;
  DigitalInput limitSwitchIntake = new DigitalInput(1);

  public IntakeIOSparkMax(int id, boolean inverted) {
    leader = new CANSparkMax(id, MotorType.kBrushless);
    encoder = leader.getEncoder();

    // leader motor is not inverted, and set follower motor to follow the leader
    leader.restoreFactoryDefaults();
    leader.setInverted(inverted);
    leader.setIdleMode(IdleMode.kBrake);
    leader.setSmartCurrentLimit(35);
    leader.burnFlash();
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.appliedVolts = leader.getAppliedOutput() * leader.getBusVoltage();
    inputs.limitSwitchIntake = !limitSwitchIntake.get();
    inputs.current = leader.getOutputCurrent();
    inputs.velocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity() / GEAR_RATIO);
  }

  @Override
  public void setVoltage(double volts) {
    // Set the voltage output for the leader motor
    leader.setVoltage(volts);
  }
}
