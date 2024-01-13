package frc.robot.subsystems;

public interface Arm {
  // gets the angle of the arm
  public default double getAngle() {
    return 0;
  }

  // sets of the angle of the arm
  public default void setAngle(double degrees) {}
}
