package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public interface Arm extends Subsystem {
  // gets the angle of the arm
  public double getAngle();

  public double getTargetAngle();

  public boolean isAtMaxLimit();

  public boolean isAtMinLimit();

  public double getVelocity();

  // sets of the angle of the arm
  public void setAngle(double degrees);

  public boolean isAtSetpoint();

  public Command getStowCommand();

  public default void add2dSim(Mechanism2d mech2d) {}
}
