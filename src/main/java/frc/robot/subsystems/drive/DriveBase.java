package frc.robot.subsystems.drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveBase extends SubsystemBase implements Drive {
  public DriveBase() {}

  public void runVelocity(ChassisSpeeds speeds) {}
  ;

  public double getMaxLinearSpeed() {
    return 0;
  }

  public double getMaxAngularSpeed() {
    return 0;
  }
}
