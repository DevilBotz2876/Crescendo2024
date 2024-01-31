package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public class ShooterIOInputs {
    public double velocityRadPerSecTop = 0.0;
    public double appliedVoltsTop = 0.0;

    public double velocityRadPerSecBottom = 0.0;
    public double appliedVoltsBottom = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ShooterIOInputs inputs) {}

  /** Run open loop at the specified voltage. */
  public default void setVoltage(double volts) {}

  /** Run closed loop at the specified velocity */
  public default void setVelocity(double velocityRadPerSec, double ffVolts) {}

  public default void stop() {}
}
