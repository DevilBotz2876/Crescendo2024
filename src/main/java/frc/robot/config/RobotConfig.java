package frc.robot.config;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.ArmIOStub;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.drive.DriveBase;
import frc.robot.subsystems.intake.IntakeBase;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.ShooterSubsystem;

/* Put all constants here with reasonable defaults */
public class RobotConfig {
  public static DriveBase drive;
  public static IntakeBase intake;
  public static ShooterSubsystem shooter;
  public static ArmSubsystem arm;
  public static SendableChooser<Command> autoChooser;

  public static class DriveConstants {
    public static double maxVelocityMetersPerSec = 4.5;
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
  }

  public static class ShooterConstants {
    /* Feedforward */
    public static double ffKs = 0.1;
    public static double ffKv = 0.0;
    public static double ffKa = 0.0;
    public static double ffKsBottom = 0.1;
    public static double ffKvBottom = 0.0;
    public static double ffKaBottom = 0.0;

    /* PID */
    public static double pidKp = 0.1;
    public static double pidKi = 0.0;
    public static double pidKd = 0.0;
    public static double pidKpBottom = 0.1;
    public static double pidKiBottom = 0.0;
    public static double pidKdBottom = 0.0;
  }

  public static class IntakeConstants {}

  public RobotConfig() {
    this(true, true, true);
  }

  public RobotConfig(boolean stubDrive, boolean stubShooter, boolean stubIntake) {
    this(stubDrive, stubShooter, stubIntake, true);
  }

  public RobotConfig(boolean stubDrive, boolean stubShooter, boolean stubIntake, boolean stubArm) {
    if (stubDrive) {
      drive = new DriveBase();
    }

    if (stubShooter) {
      shooter =
          new ShooterSubsystem(
              new ShooterIOSim(ShooterIOSim.ShooterId.SHOOTER_TOP),
              new ShooterIOSim(ShooterIOSim.ShooterId.SHOOTER_BOTTOM));
    }

    if (stubIntake) {
      intake = new IntakeBase(new IntakeIOSim());
    }

    if (stubArm) {
      arm = new ArmSubsystem(new ArmIOStub());
    }

    autoChooser = AutoBuilder.buildAutoChooser("Mobility Auto");
  }
}
