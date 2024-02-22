package frc.robot.subsystems.drive;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.config.RobotConfig.DriveConstants;
import org.littletonrobotics.junction.AutoLogOutput;

public class DriveTrain extends DriveBase {
  // Define talons
  private static final WPI_TalonSRX leftMaster = new WPI_TalonSRX(10);
  private static final WPI_TalonSRX rightMaster = new WPI_TalonSRX(11);
  private static final WPI_TalonSRX leftFollower = new WPI_TalonSRX(12);
  private static final WPI_TalonSRX rightFollower = new WPI_TalonSRX(13);
  @AutoLogOutput private double leftVoltage = 0.0;
  @AutoLogOutput private double rightVoltage = 0.0;
  @AutoLogOutput private double rightVelocity = 0.0;
  @AutoLogOutput private double leftVelocity = 0.0;

  // Define differential drive
  private final DifferentialDrive differentialDrive =
      new DifferentialDrive(leftMaster, rightMaster);

  /** Creates a new DriveTrain. */
  public DriveTrain() {
    setupTalons();
    resetEncoders();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    leftVoltage = leftMaster.getMotorOutputVoltage();
    rightVoltage = rightMaster.getMotorOutputVoltage();
  }

  @Override
  public void runVelocity(ChassisSpeeds speeds) {
    DifferentialDriveKinematics kinematics =
        new DifferentialDriveKinematics(Units.inchesToMeters(24.0));
    DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(speeds);

    leftVelocity = wheelSpeeds.leftMetersPerSecond / DriveConstants.maxVelocityMetersPerSec;
    rightVelocity = wheelSpeeds.rightMetersPerSecond / DriveConstants.maxVelocityMetersPerSec;

    differentialDrive.tankDrive(leftVelocity, rightVelocity);
  }

  private void setupTalons() {
    rightMaster.setInverted(true);
    leftMaster.setInverted(false);
    // Set the talons to follow each other
    rightFollower.follow(rightMaster);
    leftFollower.follow(leftMaster);

    // Set the follower talons to invert to match the master talons
    rightFollower.setInverted(InvertType.FollowMaster);
    leftFollower.setInverted(InvertType.FollowMaster);

    this.setTalonMode(NeutralMode.Brake);

    // Set the sensor phase of the master talons
    rightMaster.setSensorPhase(true);
    leftMaster.setSensorPhase(true);
  }

  public void resetEncoders() {
    leftMaster.setSelectedSensorPosition(0, 0, 0);
    rightMaster.setSelectedSensorPosition(0, 0, 0);
  }

  public void arcadeDrive(double speed, double rotation) {
    differentialDrive.arcadeDrive(-speed, -rotation);
  }

  public void setTalonMode(NeutralMode mode) {
    leftMaster.setNeutralMode(mode);
    rightMaster.setNeutralMode(mode);
    leftFollower.setNeutralMode(mode);
    rightFollower.setNeutralMode(mode);
  }

  public WPI_TalonSRX getLeftMaster() {
    return leftMaster;
  }

  public WPI_TalonSRX getRightMaster() {
    return rightMaster;
  }

  public WPI_TalonSRX getLeftFollower() {
    return leftFollower;
  }

  public WPI_TalonSRX getRightFollower() {
    return rightFollower;
  }

  /**
   * This returns the robot's max linear speed in meter/sec
   *
   * @return speed in meter/sec
   */
  public double getMaxLinearSpeed() {
    return DriveConstants.maxVelocityMetersPerSec;
  }

  /**
   * This returns the robot's max angular speed in radian/sec
   *
   * @return speed in radian/sec
   */
  public double getMaxAngularSpeed() {
    return DriveConstants.maxAngularVelocityRadiansSec;
  }
}
