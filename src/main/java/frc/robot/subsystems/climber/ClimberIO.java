package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
  @AutoLog
  public static class ClimberIOInputs {
    public double positionMeters = 0.0;
    public double velocityRadiansPerSecond = 0.0;
    public double appliedVolts = 0.0;
    public Object relativePositionRotations;
  }

  public void updateInputs(ClimberIOInputs inputs);

  public void setVoltage(double volts);
}
