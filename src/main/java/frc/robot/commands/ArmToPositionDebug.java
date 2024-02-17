package frc.robot.commands;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.RobotConfig.ArmConstants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmSubsystem;
import java.util.Map;

/**
 * This class uses a widget on Shuffleboard to control the arm setpoint. It is meant to be used for
 * debug/test/bring type work.
 */
public class ArmToPositionDebug extends Command {
  Arm arm;
  ShuffleboardTab tab;
  GenericEntry degreesEntry;


  public ArmToPositionDebug(Arm arm) {
    this.arm = arm;

    tab = Shuffleboard.getTab("Arm");

    degreesEntry =
        tab.add("Degrees Setpoint", 0.0)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(
                Map.of(
                    "min", ArmConstants.minAngleInDegrees, "max", ArmConstants.maxAngleInDegrees))
            .getEntry();

    addRequirements((SubsystemBase)arm);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    double setpoint = degreesEntry.getDouble(1.0);
    arm.setAngle(setpoint);
  }

  @Override
  public void end(boolean interrupted) {
    // arm.runVoltage(0);
  }
}
