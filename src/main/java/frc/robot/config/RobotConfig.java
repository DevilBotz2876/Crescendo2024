package frc.robot.config;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.auto.AutoScoreConstants;
import frc.robot.subsystems.arm.ArmIOStub;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.climber.ClimberIOStub;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.drive.DriveBase;
import frc.robot.subsystems.intake.IntakeIOStub;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterIOStub;
import frc.robot.subsystems.shooter.ShooterSubsystem;

/* Put all constants here with reasonable defaults */
public class RobotConfig {
  public static DriveBase drive;
  public static IntakeSubsystem intake;
  public static ShooterSubsystem shooter;
  public static ArmSubsystem arm;
  public static SendableChooser<Command> autoChooser;
  public static ClimberSubsystem climber;

  public static class DriveConstants {
    public static double maxVelocityMetersPerSec = 4.5;
    public static double maxAngularVelocityRadiansSec = 2 * Math.PI;
    public static double baseRadius = 0.4;
  }

  public static class ArmConstants {
    public static double absolutePositionOffset = 0; /* 0-1 */

    public static double pidKp = 0.1;
    public static double pidKi = 0.0;
    public static double pidKd = 0.0;
    public static double ffKs = 0.0;
    public static double ffKg = 0.1;
    public static double ffKv = 0.0;
    public static double ffKa = 0.0;

    public static double pidAngleErrorInDegrees = 2.0;
    public static double pidSettlingTimeInMilliseconds = 0.1;
    public static double maxAngleInDegrees = 90.0;
    public static double minAngleInDegrees = 0.0;
    public static double intakeAngleInDegrees = 1;
    public static double shooterAngleInDegrees = 45;

    public static double defaultSpeedInVolts = 6.0;
    public static double ampScoreShooterAngleInDegrees = 80;
  }

  public static class ShooterConstants {
    /* Feedforward */
    public static double ffKs = 0.088754;
    public static double ffKv = 0.029757;
    public static double ffKa = 0.01281;
    public static double ffKsBottom = 0.11831;
    public static double ffKvBottom = 0.029802;
    public static double ffKaBottom = 0.019246;

    /* PID */
    public static double pidVelocityErrorInRPMS = 20;
    public static double pidSettlingTimeInMilliseconds = 0.1;
    public static double pidKp = 0.043566;
    public static double pidKi = 0.0;
    public static double pidKd = 0.0;
    public static double pidKpBottom = 0.04467;
    public static double pidKiBottom = 0.0;
    public static double pidKdBottom = 0.0;

    public static double velocityInRPMs = 3000;
    public static double defaultSpeedInVolts = 6.0;
    public static double ampScoreVelocityInRPMs = 1000;
  }

  public static class IntakeConstants {
    public static double defaultSpeedInVolts = 6.0;
    public static double indexSpeedInVolts = 6.0;
    public static double feedSpeedInVolts = 6.0;
  }

  public static class ClimberConstants {
    public static double minPositionInRadians = 0.0;
    public static double maxPositionInRadians = 4.0;
    public static double defaultSpeedInVolts = 2.0;
    public static double autoZeroVoltage = 2.0;
    public static double autoZeroMaxCurrent = 16;
    public static double autoZeroMinVelocity = 0.2;
    public static double autoZeroExtendTimeInSeconds = 0.5;
    public static double autoZeroMaxRetractTimeInSeconds = 5.0;
    public static double autoZeroOffset =
        -0.5; // When auto-zeroing, to reduce stress on the mechanism, this is the amount we want to
    // retract the climber after auto-zeroing
  }

  public static class AutoConstants {
    /* TODO: Fill in the values for scoring here */
    public static AutoScoreConstants scoreFromSpeakerAmpSide =
        new AutoScoreConstants(0.0, 45, 3000);
    public static AutoScoreConstants scoreFromNoteAmpSide = new AutoScoreConstants(40, 45, 3000);
    public static AutoScoreConstants scoreFromSpeakerCenterSide =
        new AutoScoreConstants(0.0, 45, 3000);
    public static AutoScoreConstants scoreFromNoteCenterSide =
        new AutoScoreConstants(0.0, 40, 3000);
    public static AutoScoreConstants scoreFromSpeakerSourceSide =
        new AutoScoreConstants(0.0, 45, 3000);
    public static AutoScoreConstants scoreFromNoteSourceSide =
        new AutoScoreConstants(320.0, 30.0, 3000);
  }

  public RobotConfig() {
    this(true, true, true);
  }

  public RobotConfig(boolean stubDrive, boolean stubShooter, boolean stubIntake) {
    this(stubDrive, stubShooter, stubIntake, true, true);
  }

  public RobotConfig(
      boolean stubDrive,
      boolean stubShooter,
      boolean stubIntake,
      boolean stubArm,
      boolean stubAuto) {
    this(stubDrive, stubShooter, stubIntake, stubArm, stubAuto, true);
  }

  public RobotConfig(
      boolean stubDrive,
      boolean stubShooter,
      boolean stubIntake,
      boolean stubArm,
      boolean stubAuto,
      boolean stubClimber) {
    if (stubDrive) {
      drive = new DriveBase();
    }

    if (stubShooter) {
      shooter =
          new ShooterSubsystem(
              new ShooterIOStub(ShooterIOStub.ShooterId.SHOOTER_TOP),
              new ShooterIOStub(ShooterIOStub.ShooterId.SHOOTER_BOTTOM));
    }

    if (stubIntake) {
      intake = new IntakeSubsystem(new IntakeIOStub());
    }

    if (stubArm) {
      arm = new ArmSubsystem(new ArmIOStub());
    }

    if (stubAuto) {
      autoChooser = new SendableChooser<>();
      autoChooser.setDefaultOption("No Auto Routines Specified", Commands.none());
    }

    if (stubClimber) {
      climber = new ClimberSubsystem(new ClimberIOStub(), new ClimberIOStub());
    }
  }
}
