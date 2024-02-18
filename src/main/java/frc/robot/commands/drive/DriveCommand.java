package frc.robot.commands.drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.DriveBase;
import java.util.Map;
import java.util.function.DoubleSupplier;

public class DriveCommand extends Command {
  DriveBase drive;
  DoubleSupplier speedX;
  DoubleSupplier speedY;
  DoubleSupplier rot;
  ShuffleboardTab tab;
  GenericEntry driveSpeedEntry;
  private final SendableChooser<String> driveSpeedChooser = new SendableChooser<>();
  double xSpeed;
  double ySpeed;
  double newRot;

  GenericEntry speedLimiterEntry;
  GenericEntry turnLimiterEntry;

  public DriveCommand(
      DriveBase drive, DoubleSupplier speedX, DoubleSupplier speedY, DoubleSupplier rot) {
    this.drive = drive;
    this.speedX = speedX;
    this.speedY = speedY;
    this.rot = rot;

    tab = Shuffleboard.getTab("Drive");
    speedLimiterEntry =
        tab.add("Drive Speed Limit", 50)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", 0, "max", 100))
            .getEntry();

    turnLimiterEntry =
        tab.add("Turn (%)", 0)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", 0, "max", 100))
            .getEntry();

    turnLimiterEntry.setValue(100);
    speedLimiterEntry.setValue(50);

    driveSpeedChooser.addOption("Linear Mode", "Linear Mode");
    driveSpeedChooser.setDefaultOption("Squared Mode", "Squared Mode");
    driveSpeedChooser.addOption("Cubed Mode", "Cubed Mode");
    tab.add("Drive Response Curve", driveSpeedChooser);

    addRequirements(drive);
  }

  @Override
  public void execute() {
    switch (driveSpeedChooser.getSelected()) {
      case "Linear Mode":
        xSpeed = speedX.getAsDouble();
        ySpeed = speedY.getAsDouble();
        newRot = rot.getAsDouble();
        break;

      case "Squared Mode":
        if (speedX.getAsDouble() < 0) {
          xSpeed = Math.pow(speedX.getAsDouble(), 2) * -1;

        } else {
          xSpeed = Math.pow(speedX.getAsDouble(), 2);
        }
        if (speedY.getAsDouble() < 0) {
          ySpeed = Math.pow(speedY.getAsDouble(), 2) * -1;
        } else {
          ySpeed = Math.pow(speedY.getAsDouble(), 2);
        }
        if (rot.getAsDouble() < 0) {
          newRot = Math.pow(rot.getAsDouble(), 2) * -1;
        } else {
          newRot = Math.pow(rot.getAsDouble(), 2);
        }
        break;

      case "Cubed Mode":
        xSpeed = Math.pow(speedX.getAsDouble(), 3);
        ySpeed = Math.pow(speedY.getAsDouble(), 3);
        newRot = Math.pow(rot.getAsDouble(), 3);
        break;
    }
    ChassisSpeeds speeds =
        new ChassisSpeeds(
            xSpeed * (speedLimiterEntry.getDouble(100) / 100) * drive.getMaxLinearSpeed(),
            ySpeed * (speedLimiterEntry.getDouble(100) / 100) * drive.getMaxLinearSpeed(),
            newRot * (turnLimiterEntry.getDouble(100) / 100) * drive.getMaxAngularSpeed());

    drive.runVelocity(speeds);
  }
}
