package frc.robot.subsystems.climber;

public interface Climber {
  /** Extends climber arms min limit */
  public void extend();

  /** Retracts climber to min limit */
  public void retract();

  public void setVoltage(double volts);
}
