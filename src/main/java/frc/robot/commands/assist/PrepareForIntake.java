package frc.robot.commands.assist;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.arm.ArmToPositionTP;
import frc.robot.config.RobotConfig.ArmConstants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.intake.Intake;

public class PrepareForIntake extends ParallelCommandGroup {
  // TODO:  read arm angle and intake voltage from GUI
  NetworkTable assistGUI = NetworkTableInstance.getDefault().getTable("Shuffleboard/Assist");

  // Sets the Arm to the ideal angle for intake
  // Turns the intake motor on
  public PrepareForIntake(Arm arm, Intake intake) {

    addCommands(
        new ArmToPositionTP(
            () -> assistGUI.getEntry("Intake: Angle").getDouble(ArmConstants.intakeAngleInDegrees),
            arm));

    addCommands(new IndexPiece(intake));
  }
}
