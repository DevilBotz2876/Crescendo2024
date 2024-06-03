package frc.robot.subsystems.drive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import frc.robot.config.RobotConfig.DriveConstants;
import frc.robot.util.DevilBotState;
import java.io.File;
import java.util.Optional;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import swervelib.SwerveDrive;
import swervelib.SwerveDriveTest;
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

    AutoBuilder.configureHolonomic(
        swerveDrive::getPose, // Robot pose supplier
        swerveDrive
            ::resetOdometry, // Method to reset odometry (will be called if your auto has a starting
        // pose)
        swerveDrive::getRobotVelocity, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        swerveDrive::setChassisSpeeds, // Method that will drive the robot given ROBOT RELATIVE
        // ChassisSpeeds
        new HolonomicPathFollowerConfig(
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
    return swerveDrive.getMaximumVelocity() * 0.3;
  }

  @Override
  public double getMaxAngularSpeed() {
    return swerveDrive.getMaximumAngularVelocity() * 0.3;
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

  @Override
  public void addVisionMeasurement(
      Pose2d robotPose, double timestamp, Matrix<N3, N1> visionMeasurementStdDevs) {

    swerveDrive.addVisionMeasurement(robotPose, timestamp, visionMeasurementStdDevs);
  }

  public Optional<Double> getDistanceFromSpeaker() {
    return Optional.of(inputs.distanceFromSpeaker);
  }
}
