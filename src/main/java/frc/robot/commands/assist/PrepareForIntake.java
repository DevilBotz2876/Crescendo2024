package frc.robot.commands.assist;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.arm.ArmToPositionTP;
import frc.robot.config.RobotConfig.ArmConstants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.intake.Intake;
import java.util.function.DoubleSupplier;

public class PrepareForIntake extends SequentialCommandGroup {
  public PrepareForIntake(
      Arm arm, Intake intake, DoubleSupplier intakeAngle, DoubleSupplier intakeVoltage) {
    if (Constants.debugCommands) {
      addCommands(new PrintCommand("START: " + this.getClass().getSimpleName()));
    }
    if (intakeVoltage == null) {
      addCommands(
          new ParallelCommandGroup(new ArmToPositionTP(intakeAngle, arm)), new IndexPiece(intake));
    } else {
      addCommands(
          new ParallelCommandGroup(new ArmToPositionTP(intakeAngle, arm)),
          new IndexPiece(intake, intakeVoltage));
    }
    addCommands(new ProtectArm(arm));
    if (Constants.debugCommands) {
      addCommands(new PrintCommand("  END: " + this.getClass().getSimpleName()));
    }
  }

  // Sets the Arm to the ideal angle for intake
  // Turns the intake motor on
  public PrepareForIntake(Arm arm, Intake intake) {
    this(arm, intake, () -> ArmConstants.intakeAngleInDegrees, null);
  }
}
