package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj2.command.Command;

public interface Shooter {
  /**
   * Set the voltage of motors for the shooter.
   *
   * @param volts The volts to set. Value should be between -12.0 and 12.0.
   */
  public default void runVoltage(double volts) {}

  /** Run closed loop at the specified velocity */
  public default void runVelocity(double velocityRPM) {}

  public boolean isAtSetpoint();

  public double getCurrentSpeed();

  public double getVoltage();

  public default void turnOff() {
    runVoltage(0);
  }

  public Command getTurnOffCommand();

  public default void add2dSim(Mechanism2d mech2d) {}
  ;
}
