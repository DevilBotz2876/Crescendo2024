package frc.robot.subsystems.drive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import frc.robot.config.RobotConfig.DriveConstants;
import java.io.File;
import org.littletonrobotics.junction.AutoLogOutput;
import swervelib.SwerveDrive;
import swervelib.SwerveDriveTest;
import swervelib.parser.SwerveParser;

public class DriveSwerveYAGSL extends DriveBase {
  private final File swerveJsonDirectory;
  private SwerveDrive swerveDrive;
  @AutoLogOutput private boolean fieldOrientedDrive = false;

  public DriveSwerveYAGSL(String configPath) {
    swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), configPath);

    // SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    try {
      swerveDrive =
          new SwerveParser(swerveJsonDirectory)
              .createSwerveDrive(DriveConstants.maxVelocityMetersPerSec);
    } catch (Exception e) {
      throw new RuntimeException(e);
    }

    AutoBuilder.configureHolonomic(
        swerveDrive::getPose, // Robot pose supplier
        swerveDrive
            ::resetOdometry, // Method to reset odometry (will be called if your auto has a starting
        // pose)
        swerveDrive::getRobotVelocity, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        swerveDrive::setChassisSpeeds, // Method that will drive the robot given ROBOT RELATIVE
        // ChassisSpeeds
        new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in
            // your Constants class
            new PIDConstants(0.0020645, 0.0, 0.0), // Translation PID constants
            new PIDConstants(0.01, 0.0, 0.00), // Rotation PID constants
            swerveDrive.getMaximumVelocity(), // Max module speed, in m/s
            swerveDrive.swerveDriveConfiguration
                .getDriveBaseRadiusMeters(), // Drive base radius in meters. Distance from robot
            // center to furthest module.
            new ReplanningConfig() // Default path replanning config. See the API for the options
            // here
            ),
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this // Reference to this subsystem to set requirements
        );
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

  public void resetOdometry() {
    swerveDrive.resetOdometry(new Pose2d());
  }

  @Override
  public void setPoseToMatchField() {
    swerveDrive.resetOdometry(swerveDrive.field.getRobotPose());
  }

  /**
   * Command to characterize the robot drive motors using SysId
   *
   * @return SysId Drive Command
   */
  public Command sysIdDriveMotorCommand() {
    return SwerveDriveTest.generateSysIdCommand(
        SwerveDriveTest.setDriveSysIdRoutine(new Config(), this, swerveDrive, 12), 3.0, 5.0, 3.0);
  }

  /**
   * Command to characterize the robot angle motors using SysId
   *
   * @return SysId Angle Command
   */
  public Command sysIdAngleMotorCommand() {
    return SwerveDriveTest.generateSysIdCommand(
        SwerveDriveTest.setAngleSysIdRoutine(new Config(), this, swerveDrive), 3.0, 5.0, 3.0);
  }

  @Override
  public double getAngle() {

    return swerveDrive.getPose().getRotation().getDegrees();
  }
}
