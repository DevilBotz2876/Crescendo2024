package frc.robot.subsystems;

public interface Arm {
  // gets the angle of the arm.
  public default void getAngle() {}

  // sets of the angle of the arm.
  public default double setAngle() {
    return 0;
  }
}
