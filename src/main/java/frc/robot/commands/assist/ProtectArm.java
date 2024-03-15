package frc.robot.commands.assist;

import frc.robot.commands.arm.ArmToPositionPP;
import frc.robot.config.RobotConfig;
import frc.robot.subsystems.arm.Arm;

public class ProtectArm extends ArmToPositionPP {
  ProtectArm(Arm arm) {
    super(() -> RobotConfig.ArmConstants.stowIntakeAngleInDegrees, arm);
  }
}
