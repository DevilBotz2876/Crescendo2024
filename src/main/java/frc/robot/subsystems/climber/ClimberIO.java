package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
  @AutoLog
  public static class ClimberIOInputs {
    public double positionMeters = 0.0;
    public double velocityMetersPerSecond = 0.0;
    public double appliedVolts = 0.0;
  }

  public void updateInputs(ClimberIOInputs inputs);

  public void setVoltage(double volts);
}
