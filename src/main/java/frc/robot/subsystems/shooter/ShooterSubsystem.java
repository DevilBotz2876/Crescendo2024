package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class ShooterSubsystem extends SubsystemBase implements Shooter {
  ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
  @AutoLogOutput private double voltage;
  @AutoLogOutput private double setPoint;

  public ShooterSubsystem(ShooterIO io) {
    this.io = io;
    voltage = 0;
    setPoint = 0;
  }

  @Override
  // Disable the shooter
  public void disable() {
    voltage = 0;
    setPoint = 0;
    io.setVoltage(voltage);
  }

  @Override
  // Enable the shooter
  public void enable() {
    io.setVoltage(voltage);
  }

  @Override
  // Sets the voltage to volts. the volts value is -12 to 12
  public void setVoltage(double volts) {
    voltage = volts;
  }

  @Override
  public void setVelocity(double setPoint) {
    this.setPoint = setPoint;
  }

  @Override
  public double getVoltage() {
    return inputs.appliedVoltsTop;
  }

  @Override
  public double getCurrentSpeed() {
    return inputs.velocityRadPerSecTop;
  }

  @Override
  public void periodic() {
    // Updates the inputs
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
    SmartDashboard.putNumber("Shooter/TopRPM", inputs.velocityRadPerSecTop);
    SmartDashboard.putNumber("Shooter/BottomRPM", inputs.velocityRadPerSecBottom);
  }
}
