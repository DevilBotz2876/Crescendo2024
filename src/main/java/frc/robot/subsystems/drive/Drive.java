package frc.robot.subsystems.drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public interface Drive {
  /**
   * Sets the desired chassis speed
   *
   * @param speeds specifies the setpoint speed for the chassis
   */
  public void runVelocity(ChassisSpeeds speeds);

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

  /*
  public void stop();
  public Pose2d getPose();
  public void setPose(Pose2d Pose2d);
  public Rotation2d getRotation();
  */

}
