package frc.robot.subsystems.climber;

public class ClimberIOStub implements ClimberIO {
  private double voltage = 0.0;
  private double position = 0.0;

  public void updateInputs(ClimberIOInputs inputs) {
    if (voltage > 0) {
      inputs.velocityMetersPerSecond = .25;
    } else if (voltage < 0) {
      inputs.velocityMetersPerSecond = -.25;
    } else {
      inputs.velocityMetersPerSecond = 0;
    }

    position += inputs.velocityMetersPerSecond / 20;

    inputs.positionMeters = position;
    inputs.appliedVolts = voltage;
  }

  public void setVoltage(double volts) {
    this.voltage = volts;
  }
}
