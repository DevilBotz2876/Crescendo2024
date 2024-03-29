package frc.robot.subsystems.drive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import frc.robot.config.RobotConfig.DriveConstants;
import frc.robot.util.DevilBotState;
import java.io.File;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import swervelib.SwerveDrive;
import swervelib.SwerveDriveTest;
import swervelib.parser.PIDFConfig;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;

public class DriveSwerveYAGSL extends DriveBase {
  private final File swerveJsonDirectory;
  private SwerveDrive swerveDrive;
  @AutoLogOutput private boolean fieldOrientedDrive = false;

  // @AutoLogOutput
  DriveIO io = new DriveIO();
  private final DriveIOInputsAutoLogged inputs = new DriveIOInputsAutoLogged();

  public DriveSwerveYAGSL(String configPath) {
    swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), configPath);

    // SwerveDriveTelemetry.verbosity = TelemetryVerbosity.LOW;
    try {
      swerveDrive =
          new SwerveParser(swerveJsonDirectory)
              .createSwerveDrive(DriveConstants.maxVelocityMetersPerSec);
      swerveDrive.setCosineCompensator(!SwerveDriveTelemetry.isSimulation);

      swerveDrive
          .getSwerveController()
          .addSlewRateLimiters(
              new SlewRateLimiter(DriveConstants.slewRateLimiterX),
              new SlewRateLimiter(DriveConstants.slewRateLimiterY),
              new SlewRateLimiter(DriveConstants.slewRateLimiterAngle));

      swerveDrive.setMotorIdleMode(true);
    } catch (Exception e) {
      throw new RuntimeException(e);
    }

    PIDFConfig drivePIDF = swerveDrive.getModules()[0].getDrivePIDF();
    PIDFConfig anglePIDF = swerveDrive.getModules()[0].getAnglePIDF();

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
            new PIDConstants(drivePIDF.p, drivePIDF.i, drivePIDF.d), // Translation PID constants
            new PIDConstants(anglePIDF.p, anglePIDF.i, anglePIDF.d), // Rotation PID constants
            swerveDrive.getMaximumVelocity(), // Max module speed, in m/s
            swerveDrive.swerveDriveConfiguration
                .getDriveBaseRadiusMeters(), // Drive base radius in meters. Distance from robot
            // center to furthest module.
            new ReplanningConfig() // Default path replanning config. See the API for the options
            // here
            ),
        () -> DevilBotState.isRedAlliance(),
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
  public Pose2d getPose() {
    return swerveDrive.getPose();
  }

  @Override
  public double getAngle() {
    return swerveDrive.getOdometryHeading().getDegrees();
  }

  @Override
  public void lockPose() {
    swerveDrive.lockPose();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs, swerveDrive);
    Logger.processInputs("Drive", inputs);
  }
}
