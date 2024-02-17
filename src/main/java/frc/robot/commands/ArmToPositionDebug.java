package frc.robot.commands;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.config.RobotConfig.ArmConstants;
import frc.robot.subsystems.arm.ArmSubsystem;
import java.util.Map;

/**
 * This class uses a widget on Shuffleboard to control the arm setpoint. It is meant to be used for
 * debug/test/bring type work.
 */
public class ArmToPositionDebug extends Command {
  ArmSubsystem arm;
  ShuffleboardTab tab;
  GenericEntry degreesEntry;

  // double setpoint;
  // double timeMS;

  public ArmToPositionDebug(ArmSubsystem arm) {
    this.arm = arm;

    tab = Shuffleboard.getTab("Arm");

    degreesEntry =
        tab.add("Degrees Setpoint", 0.0)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(
                Map.of(
                    "min", ArmConstants.minAngleInDegrees, "max", ArmConstants.maxAngleInDegrees))
            .getEntry();

    addRequirements(arm);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {

    double setpoint = degreesEntry.getDouble(1.0);
    arm.setAngle(setpoint);
  }

  // @Override
  // public boolean isFinished() {

  //   if (arm.getAngle() > setpoint - Constants.armErroDegrees && arm.getAngle() < setpoint +
  // Constants.armErroDegrees) {
  //     timeMS += 20.0;
  //     if (timeMS == 1000) {
  //       SmartDashboard.putBoolean("Arm/ArmToPositionDebug/isFinished", true);
  //       return true;
  //     }
  //   } else {
  //     timeMS = 0.0;
  //   }
  //   SmartDashboard.putBoolean("Arm/ArmToPositionDebug/isFinished", false);
  //   return false;
  // }

  @Override
  public void end(boolean interrupted) {
    // arm.runVoltage(0);
  }
}
