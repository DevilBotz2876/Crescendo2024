package frc.robot.subsystems.drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveBase extends SubsystemBase implements Drive {
  @Override
  public void runVelocity(ChassisSpeeds velocity) {}

  @Override
  public double getMaxLinearSpeed() {
    return 0;
  }

  @Override
  public double getMaxAngularSpeed() {
    return 0;
  }
}
