package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class ShooterSubsystem extends SubsystemBase implements Shooter {
  ShooterIO io;
  private final SimpleMotorFeedforward ffModel;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
  @AutoLogOutput private double voltage;
  @AutoLogOutput private double velocityRPM;
  @AutoLogOutput private double targetVelocityRadPerSec;

  public ShooterSubsystem(ShooterIO io) {
    this.io = io;
    // TODO: These are sample values.  Need to run sysid on shooter and get real values.
    ffModel = new SimpleMotorFeedforward(0.1, 0.05);
    voltage = 0;
    velocityRPM = 0.0;
  }

  @Override
  // Sets the voltage to volts. the volts value is -12 to 12
  public void runVoltage(double volts) {
    voltage = volts;
    targetVelocityRadPerSec = 0;
    io.setVoltage(voltage);
  }

  @Override
  public void runVelocity(double velocityRPM) {
    voltage = 0;
    this.velocityRPM = velocityRPM;
    targetVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM);

    io.setVelocity(targetVelocityRadPerSec, ffModel.calculate(targetVelocityRadPerSec));
  }

  @Override
  public double getVoltage() {
    return inputs.appliedVolts;
  }

  @Override
  public double getCurrentSpeed() {
    return inputs.velocityRadPerSec;
  }

  @Override
  public void periodic() {
    // Updates the inputs
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
  }
}
