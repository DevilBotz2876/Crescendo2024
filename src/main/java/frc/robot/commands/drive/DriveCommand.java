package frc.robot.commands.drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
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
  GenericEntry speedLimiterEntry;

  public DriveCommand(
      DriveBase drive, DoubleSupplier speedX, DoubleSupplier speedY, DoubleSupplier rot) {
    this.drive = drive;
    this.speedX = speedX;
    this.speedY = speedY;
    this.rot = rot;

    tab = Shuffleboard.getTab("Assist");
    speedLimiterEntry =
        tab.add("Drive Speed Limit", 0)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", 0, "max", 100))
            .getEntry();

    speedLimiterEntry.setValue(50);

    addRequirements(drive);
  }

  @Override
  public void execute() {
    ChassisSpeeds speeds =
        new ChassisSpeeds(
            speedX.getAsDouble()
                * (speedLimiterEntry.getDouble(100) / 100)
                * drive.getMaxLinearSpeed(),
            speedY.getAsDouble()
                * (speedLimiterEntry.getDouble(100) / 100)
                * drive.getMaxLinearSpeed(),
            rot.getAsDouble()
                * (speedLimiterEntry.getDouble(100) / 100)
                * drive.getMaxAngularSpeed());

    drive.runVelocity(speeds);
  }
}
