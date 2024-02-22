package frc.robot.commands.shooter;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.RobotConfig.ArmConstants;
import frc.robot.config.RobotConfig.IntakeConstants;
import frc.robot.config.RobotConfig.ShooterConstants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

public class TestShooterAngle extends Command {
  Shooter shooter;
  Arm arm;
  Intake intake;
  NetworkTableEntry ShooterVelocity;
  NetworkTableEntry IntakeVoltage;
  NetworkTableEntry ArmAngle;
  NetworkTable assistGUI = NetworkTableInstance.getDefault().getTable("Shuffleboard/Assist");

  public TestShooterAngle(Shooter shooter, Intake intake, Arm arm) {
    this.shooter = shooter;
    this.intake = intake;
    this.arm = arm;
    this.ShooterVelocity = assistGUI.getEntry("Shooter Velocity");
    this.IntakeVoltage = assistGUI.getEntry("Feed Piece Volts");
    this.ArmAngle = assistGUI.getEntry("Shooter Angle");
    addRequirements((SubsystemBase) shooter);
    addRequirements((SubsystemBase) intake);
    addRequirements((SubsystemBase) arm);
  }

  @Override
  public void execute() {
    shooter.runVelocity(ShooterVelocity.getDouble(ShooterConstants.velocityInRPMs));
    intake.runVoltage(IntakeVoltage.getDouble(IntakeConstants.feedSpeedInVolts));
    arm.setAngle(ArmAngle.getDouble(ArmConstants.shooterAngleInDegrees));
  }

  public void end(boolean interrupted) {
    shooter.runVelocity(0);
    intake.runVoltage(0);
  }
}
