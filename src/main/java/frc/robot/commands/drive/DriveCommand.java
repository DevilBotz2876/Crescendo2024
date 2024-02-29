package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.config.RobotConfig.DriveConstants;
import frc.robot.subsystems.drive.DriveBase;
import java.util.Map;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class DriveCommand extends Command {
  DriveBase drive;
  DoubleSupplier speedX;
  DoubleSupplier speedY;
  DoubleSupplier rot;
  Supplier<Optional<Double>> autoRot;
  ShuffleboardTab tab;
  GenericEntry driveSpeedEntry;
  private static SendableChooser<String> driveSpeedChooser = null;
  double xSpeed;
  double ySpeed;
  double newRot;
  double speedLimiter;
  double turnLimiter;
  PIDController turnPID;

  static GenericEntry speedLimiterEntry = null;
  static GenericEntry turnLimiterEntry = null;

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
            DriveConstants.anglePidKp, DriveConstants.anglePidKi, DriveConstants.anglePidKd);
    turnPID.enableContinuousInput(0, 360);

    tab = Shuffleboard.getTab("Drive");
    if (speedLimiterEntry == null) {
      speedLimiterEntry =
          tab.add("Drive Speed Limit", 100)
              .withWidget(BuiltInWidgets.kNumberSlider)
              .withProperties(Map.of("min", 0, "max", 100))
              .getEntry();
    }

    if (turnLimiterEntry == null) {
      turnLimiterEntry =
          tab.add("Drive Turn Limit", 100)
              .withWidget(BuiltInWidgets.kNumberSlider)
              .withProperties(Map.of("min", 0, "max", 100))
              .getEntry();
    }

    if (driveSpeedChooser == null) {
      driveSpeedChooser = new SendableChooser<>();
      driveSpeedChooser.addOption("Linear Mode", "Linear Mode");
      driveSpeedChooser.setDefaultOption("Squared Mode", "Squared Mode");
      driveSpeedChooser.addOption("Cubed Mode", "Cubed Mode");
      tab.add("Drive Response Curve", driveSpeedChooser);
    }

    addRequirements(drive);
  }

  public DriveCommand(
      DriveBase drive, DoubleSupplier speedX, DoubleSupplier speedY, DoubleSupplier rot) {
    this(drive, speedX, speedY, rot, () -> Optional.empty());
  }

  @Override
  public void execute() {
    String driveSpeedSelceted = driveSpeedChooser.getSelected();
    xSpeed = speedX.getAsDouble();
    ySpeed = speedY.getAsDouble();
    newRot = rot.getAsDouble();

    speedLimiter = speedLimiterEntry.getDouble(100);
    turnLimiter = turnLimiterEntry.getDouble(100);

    switch (driveSpeedSelceted) {
      case "Linear Mode":
        break;

      case "Squared Mode":
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
        break;

      case "Cubed Mode":
        xSpeed = Math.pow(xSpeed, 3);
        ySpeed = Math.pow(xSpeed, 3);
        newRot = Math.pow(newRot, 3);
        break;
    }

    if (this.autoRot.get().isPresent()) {
      double currentAngle = drive.getAngle();
      newRot = turnPID.calculate(currentAngle, currentAngle - this.autoRot.get().get());
      System.out.println("Auto Rotate");
    }

    ChassisSpeeds speeds =
        new ChassisSpeeds(
            xSpeed * (speedLimiter / 100) * drive.getMaxLinearSpeed(),
            ySpeed * (speedLimiter / 100) * drive.getMaxLinearSpeed(),
            newRot * (turnLimiter / 100) * drive.getMaxAngularSpeed());

    drive.runVelocity(speeds);
  }
}
