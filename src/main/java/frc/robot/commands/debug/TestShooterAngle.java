package frc.robot.commands.debug;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import java.util.function.DoubleSupplier;

public class TestShooterAngle extends Command {
  Shooter shooter;
  Arm arm;
  Intake intake;

  DoubleSupplier shooterVelocity;
  DoubleSupplier intakeVoltage;
  DoubleSupplier armAngle;

  public TestShooterAngle(
      Shooter shooter,
      Intake intake,
      Arm arm,
      DoubleSupplier shooterVelocity,
      DoubleSupplier intakeVoltage,
      DoubleSupplier armAngle) {
    this.shooter = shooter;
    this.intake = intake;
    this.arm = arm;
    this.shooterVelocity = shooterVelocity;
    this.intakeVoltage = intakeVoltage;
    this.armAngle = armAngle;

    addRequirements((Subsystem) shooter);
    addRequirements((Subsystem) intake);
    addRequirements((Subsystem) arm);
  }

  @Override
  public void execute() {
    shooter.runVelocity(shooterVelocity.getAsDouble());
    intake.runVoltage(intakeVoltage.getAsDouble());
    arm.setAngle(armAngle.getAsDouble());
  }

  public void end(boolean interrupted) {
    shooter.runVelocity(0);
    intake.runVoltage(0);
  }
}
