package frc.robot.subsystems.drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;

public class DriveBase extends SubsystemBase implements Drive {
  protected static final DifferentialDriveKinematics kinematics =
      new DifferentialDriveKinematics(Units.inchesToMeters(27.0));

  @AutoLogOutput protected double leftVelocityMetersPerSecond;
  @AutoLogOutput protected double rightVelocityMetersPerSecond;

  public DriveBase() {}

  public void runVelocity(ChassisSpeeds chassisSpeeds) {
    // Convert to wheel speeds
    DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(chassisSpeeds);

    leftVelocityMetersPerSecond = wheelSpeeds.leftMetersPerSecond;
    rightVelocityMetersPerSecond = wheelSpeeds.rightMetersPerSecond;
  }

  public double getMaxLinearSpeed() {
    return 5;
  }

  public double getMaxAngularSpeed() {
    return 5;
  }
}
