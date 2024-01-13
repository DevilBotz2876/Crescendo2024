package frc.robot.subsystems;

public interface Shooter {
    
  // Sets enable(true/false) to be used for the motors. on=true and off=false
  public default void setEnabled(boolean isEnabled) {}

  // Get enable(true/false). on=true and off=false
  public default boolean getEnabled() {
    return false;
  }
// Sets the voltage of the motors.
  public default void setVoltage(double voltage) {}
  
// gets the voltage of the motors.
  public default double getVoltage(){
    return 0;
  }


}
