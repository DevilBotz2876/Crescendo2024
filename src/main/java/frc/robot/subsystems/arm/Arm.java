package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.config.RobotConfig.ArmConstants;

public interface Arm extends Subsystem {
  // gets the angle of the arm
  public double getAngle();

  public double getTargetAngle();

  // sets of the angle of the arm
  public void setAngle(double degrees);

  public default double getVelocity() {
    return 0;
  }

  public default double getRelativeAngle() {
    return getAngle();
  }

  public default boolean isAbsoluteEncoderConnected() {
    return true;
  }

  public default boolean isAtMaxLimit() {
    return false;
  }

  public default boolean isAtMinLimit() {
    return false;
  }

  public default void stow() {
    setAngle(ArmConstants.stowIntakeAngleInDegrees);
  }

  public Command getStowCommand();

  public default void add2dSim(Mechanism2d mech2d) {}
}
