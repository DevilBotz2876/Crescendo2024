package frc.robot.subsystems.drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

public interface Drive {
  /**
   * Sets the desired chassis speed
   *
   * @param velocity specifies the setpoint velocity for the chassis
   */
  public void runVelocity(ChassisSpeeds velocity);

  /**
   * This returns the robot's max linear speed in meter/sec
   *
   * @return speed in meter/sec
   */
  public double getMaxLinearSpeed();

  /**
   * This returns the robot's max angular speed in radian/sec
   *
   * @return speed in radian/sec
   */
  public double getMaxAngularSpeed();

  public default void setFieldOrientedDrive(boolean enable) {}
  ;

  public default boolean isFieldOrientedDrive() {
    return false;
  }

  public default void resetOdometry() {}
  ;

  public default void setPoseToMatchField() {}

  /**
   * Command to characterize the robot drive motors using SysId
   *
   * @return SysId Drive Command
   */
  public default Command sysIdDriveMotorCommand() {
    return null;
  }

  /**
   * Command to characterize the robot angle motors using SysId
   *
   * @return SysId Angle Command
   */
  public default Command sysIdAngleMotorCommand() {
    return null;
  }
}
