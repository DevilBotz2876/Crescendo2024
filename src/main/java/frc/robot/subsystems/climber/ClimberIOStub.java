package frc.robot.subsystems.climber;

public class ClimberIOStub implements ClimberIO {
  private double voltage = 0.0;
  private double position = 0.0;
  private double velocityRadiansPerSec = 1;

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    if (voltage > 0) {
      inputs.velocityRadiansPerSecond = velocityRadiansPerSec;
    } else if (voltage < 0) {
      inputs.velocityRadiansPerSecond = -velocityRadiansPerSec;
    } else {
      inputs.velocityRadiansPerSecond = 0;
    }

    position += inputs.velocityRadiansPerSecond / 50;

    inputs.positionRadians = position;
    inputs.appliedVolts = voltage;
  }

  @Override
  public void setVoltage(double volts) {
    this.voltage = volts;
  }

  @Override
  public void resetPosition() {
    position = 0.0;
  }
}
