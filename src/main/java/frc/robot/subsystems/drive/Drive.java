package frc.robot.subsystems.drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public interface Drive {
  public void runVelocity(ChassisSpeeds speeds);

  public double getMaxLinearSpeed();

  public double getMaxAngularSpeed();

  /*
  public void stop();

  public Pose2d getPose();

  public void setPose(Pose2d Pose2d);

  public Rotation2d getRotation();
  */
}
