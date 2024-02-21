package frc.robot.subsystems.climber;

public class ClimberIOStub implements ClimberIO {
  private double voltage = 0.0;
  private double position = 0.0;

  public void updateInputs(ClimberIOInputs inputs) {
    if (voltage > 0) {
      inputs.velocityRadiansPerSecond = .25;
    } else if (voltage < 0) {
      inputs.velocityRadiansPerSecond = -.25;
    } else {
      inputs.velocityRadiansPerSecond = 0;
    }

    position += inputs.velocityRadiansPerSecond / 50;

    inputs.positionMeters = position;
    inputs.appliedVolts = voltage;
  }

  public void setVoltage(double volts) {
    this.voltage = volts;
  }
}
