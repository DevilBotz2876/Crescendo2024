package frc.robot.commands.assist;

import frc.robot.commands.arm.ArmToPositionTP;
import frc.robot.config.RobotConfig;
import frc.robot.subsystems.arm.Arm;

public class ProtectArm extends ArmToPositionTP {
  ProtectArm(Arm arm) {
    super(() -> RobotConfig.ArmConstants.stowIntakeAngleInDegrees, arm);
  }
}
