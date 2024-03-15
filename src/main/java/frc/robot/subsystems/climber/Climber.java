package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj2.command.Command;

public interface Climber {
  public void runVoltage(double volts);

  public default void runVoltageLeft(double volts) {}

  public default void runVoltageRight(double volts) {}

  public void resetPosition();

  public default void enableLimits(boolean enable) {}

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
