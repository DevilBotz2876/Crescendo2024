package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj2.command.Command;

public interface Climber {
  /** Extends climber arms min limit */
  public void extend();

  /** Retracts climber to min limit */
  public void retract();

  public void runVoltage(double volts);

  public default void runVoltageLeft(double volts) {}

  public default void runVoltageRight(double volts) {}

  public void resetPosition();

  public void overridePosition(double leftpos, double rightPos);

  public default void autoZeroMode(boolean enable) {}

  public default void enableLimits(boolean enable) {}

  public boolean isExtending();

  public default double getCurrentPositionLeft() {
    return 0;
  }

  public default double getCurrentPositionRight() {
    return 0;
  }

  public default boolean isAtMaxLimitLeft() {
    return false;
  }

  public default boolean isAtMinLimitLeft() {
    return false;
  }

  public default boolean isAtMaxLimitRight() {
    return false;
  }

  public default boolean isAtMinLimitRight() {
    return false;
  }

  public Command getExtendCommand();

  public Command getRetractCommand();

  public default void add2dSim(Mechanism2d mech2d) {}
  ;
}
