package frc.robot.subsystems.drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.File;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;

public class DriveSwerveYAGSL extends DriveBase {
  private final double maximumSpeed =
      Units.feetToMeters(2); // * TODO: Calculate actual max speed */
  private final File swerveJsonDirectory =
      new File(Filesystem.getDeployDirectory(), "swervePracticeBot");
  private SwerveDrive swerveDrive;
  private boolean fieldOrientedDrive = false;

  public DriveSwerveYAGSL() {
    try {
      swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(maximumSpeed);
    } catch (Exception e) {
      throw new RuntimeException(e);
    }
  }

  @Override
  public void runVelocity(ChassisSpeeds velocity) {
    if (fieldOrientedDrive) {
      swerveDrive.driveFieldOriented(velocity);
    } else {
      swerveDrive.drive(velocity);
    }
  }

  @Override
  public double getMaxLinearSpeed() {
    return swerveDrive.getMaximumVelocity();
  }

  @Override
  public double getMaxAngularSpeed() {
    return swerveDrive.getMaximumAngularVelocity();
  }

  public void setFieldOrientedDrive(boolean enable) {
    fieldOrientedDrive = enable;
  }

  public boolean isFieldOrientedDrive() {
    return fieldOrientedDrive;
  }
}
