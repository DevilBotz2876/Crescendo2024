package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.config.RobotConfig.DriveConstants;
import frc.robot.subsystems.drive.DriveBase;
import frc.robot.util.DevilBotState;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class DriveCommand extends Command {
  DriveBase drive;
  DoubleSupplier speedX;
  DoubleSupplier speedY;
  DoubleSupplier rot;
  Supplier<Optional<Double>> autoRot;
  double xSpeed;
  double ySpeed;
  double newRot;
  PIDController turnPID;

  public DriveCommand(
      DriveBase drive,
      DoubleSupplier speedX,
      DoubleSupplier speedY,
      DoubleSupplier rot,
      Supplier<Optional<Double>> autoRot) {
    this.drive = drive;
    this.speedX = speedX;
    this.speedY = speedY;
    this.rot = rot;
    this.autoRot = autoRot;

    // Wild guess at P constant.
    turnPID =
        new PIDController(
            DriveConstants.rotatePidKp, DriveConstants.rotatePidKi, DriveConstants.rotatePidKd);
    turnPID.enableContinuousInput(0, 360);

    addRequirements(drive);
  }

  public DriveCommand(
      DriveBase drive, DoubleSupplier speedX, DoubleSupplier speedY, DoubleSupplier rot) {
    this(drive, speedX, speedY, rot, () -> Optional.empty());
  }

  @Override
  public void execute() {
    xSpeed = speedX.getAsDouble();
    ySpeed = speedY.getAsDouble();
    newRot = rot.getAsDouble();

    //    case "Squared Mode":
    if (xSpeed < 0) {
      xSpeed = Math.pow(xSpeed, 2) * -1;

    } else {
      xSpeed = Math.pow(xSpeed, 2);
    }
    if (ySpeed < 0) {
      ySpeed = Math.pow(ySpeed, 2) * -1;
    } else {
      ySpeed = Math.pow(ySpeed, 2);
    }
    if (newRot < 0) {
      newRot = Math.pow(newRot, 2) * -1;
    } else {
      newRot = Math.pow(newRot, 2);
    }

    if (this.autoRot.get().isPresent()) {
      double currentAngle = drive.getAngle();
      newRot = turnPID.calculate(currentAngle, currentAngle - this.autoRot.get().get());
    }

    /* Invert strafe controls if Field Oriented and on the Red Alliance */
    if (DevilBotState.isFieldOriented() && DevilBotState.isRedAlliance()) {
      xSpeed *= -1;
      ySpeed *= -1;
    }

    ChassisSpeeds speeds =
        new ChassisSpeeds(
            xSpeed * drive.getMaxLinearSpeed(),
            ySpeed * drive.getMaxLinearSpeed(),
            newRot * drive.getMaxAngularSpeed());

    drive.runVelocity(speeds);
  }
}
