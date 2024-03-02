package frc.robot.subsystems.arm;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Subsystem;

public interface Arm extends Subsystem{
  // gets the angle of the arm
  public default double getAngle() {
    return 0;
  }

  // sets of the angle of the arm
  public default void setAngle(double degrees) {}

  public default boolean isAbsoluteEncoderConnected() {
    return true;
  }

  // Set the new/requested state of the arm.
  public default void setState(TrapezoidProfile.State state) {}

  // Get the current state of the arm
  public default TrapezoidProfile.State getState() {
    return new TrapezoidProfile.State();
  }

  // Get trapezoid profile constraints that limit how fast the arm moves.
  public default TrapezoidProfile.Constraints getConstraints() {
    return new TrapezoidProfile.Constraints(1, 2);
  }
}
