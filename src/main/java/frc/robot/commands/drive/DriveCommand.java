package frc.robot.commands.drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  double speedLimiter;

  GenericEntry speedLimiterEntry;

  public DriveCommand(
      DriveBase drive, DoubleSupplier speedX, DoubleSupplier speedY, DoubleSupplier rot) {
    this.drive = drive;
    this.speedX = speedX;
    this.speedY = speedY;
    this.rot = rot;

    SmartDashboard.putData(driveSpeedChooser);
    tab = Shuffleboard.getTab("Drive");
    speedLimiterEntry =
        tab.add("Drive Speed Limit", 50)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", 0, "max", 100))
            .getEntry();

    driveSpeedChooser.addOption("Linear Mode", "Linear Mode");
    driveSpeedChooser.setDefaultOption("Squared Mode", "Squared Mode");
    driveSpeedChooser.addOption("Cubed Mode", "Cubed Mode");
    tab.add("Drive Response Curve", driveSpeedChooser);

    addRequirements(drive);
  }

  @Override
  public void execute() {
    String driveSpeedSelceted = driveSpeedChooser.getSelected();
    xSpeed = speedX.getAsDouble();
    ySpeed = speedY.getAsDouble();
    newRot = rot.getAsDouble();
    speedLimiter = speedLimiterEntry.getDouble(100);

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
    ChassisSpeeds speeds =
        new ChassisSpeeds(
            xSpeed * (speedLimiter / 100) * drive.getMaxLinearSpeed(),
            ySpeed * (speedLimiter / 100) * drive.getMaxLinearSpeed(),
            newRot * (speedLimiter / 100) * drive.getMaxAngularSpeed());

    drive.runVelocity(speeds);
  }
}
