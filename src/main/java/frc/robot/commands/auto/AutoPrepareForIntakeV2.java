package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.arm.ArmToPositionTP;
import frc.robot.config.RobotConfig;
import frc.robot.config.RobotConfig.ArmConstants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.intake.Intake;

public class AutoPrepareForIntakeV2 extends SequentialCommandGroup {
  public AutoPrepareForIntakeV2(Arm arm, Intake intake) {
    if (Constants.debugCommands) {
      addCommands(new PrintCommand("START: " + this.getClass().getSimpleName()));
    }
    addCommands(
        new ParallelCommandGroup(
            new InstantCommand(
                () -> intake.runVoltage(RobotConfig.IntakeConstants.defaultSpeedInVolts)),
            new ArmToPositionTP(
                () -> ArmConstants.intakeAngleInDegrees, arm) // Move arm to intake position
            ));

    if (Constants.debugCommands) {
      addCommands(new PrintCommand("  END: " + this.getClass().getSimpleName()));
    }
  }
}
