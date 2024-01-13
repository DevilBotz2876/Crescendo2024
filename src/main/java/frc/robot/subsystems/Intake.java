package frc.robot.subsystems;

public interface Intake {
  // Sets the Intake to a mode(in/out)
  public default void setMode(String direction) {}

  // Gets the Intake mode (in/out)
  public default String getMode() {
    return null;
  }

  // Sets enable(true/false) to be used for the motors. on=true and off=false
  public default void setEnabled(boolean isEnabled) {}

  // Gets enable or disable Intake value.
  public default boolean getEnabled() {
    return false;
  }
}
