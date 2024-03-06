package frc.robot.controls;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.config.RobotConfig;

public class SysIdControls {
  public static void setupGUI() {
    int rowIndex = 0;
    int colIndex = 0;

    ShuffleboardTab sysIdTestTab = Shuffleboard.getTab("SysId");

    sysIdTestTab
        .add(
            "Arm: Quasistatic Forward",
            RobotConfig.arm.sysIdQuasistatic(SysIdRoutine.Direction.kForward))
        .withPosition(colIndex, rowIndex++)
        .withSize(2, 1);
    sysIdTestTab
        .add(
            "Arm: Quasistatic Reverse",
            RobotConfig.arm.sysIdQuasistatic(SysIdRoutine.Direction.kReverse))
        .withPosition(colIndex, rowIndex++)
        .withSize(2, 1);
    sysIdTestTab
        .add("Arm: Dynamic Forward", RobotConfig.arm.sysIdDynamic(SysIdRoutine.Direction.kForward))
        .withPosition(colIndex, rowIndex++)
        .withSize(2, 1);
    sysIdTestTab
        .add("Arm: Dynamic Reverse", RobotConfig.arm.sysIdDynamic(SysIdRoutine.Direction.kReverse))
        .withPosition(colIndex, rowIndex++)
        .withSize(2, 1);

    rowIndex = 0;
    colIndex += 2;
    sysIdTestTab
        .add(
            "Shooter: Quasistatic Forward",
            RobotConfig.shooter.sysIdQuasistatic(SysIdRoutine.Direction.kForward))
        .withPosition(colIndex, rowIndex++)
        .withSize(2, 1);
    sysIdTestTab
        .add(
            "Shooter: Quasistatic Reverse",
            RobotConfig.shooter.sysIdQuasistatic(SysIdRoutine.Direction.kReverse))
        .withPosition(colIndex, rowIndex++)
        .withSize(2, 1);
    sysIdTestTab
        .add(
            "Shooter: Dynamic Forward",
            RobotConfig.shooter.sysIdDynamic(SysIdRoutine.Direction.kForward))
        .withPosition(colIndex, rowIndex++)
        .withSize(2, 1);
    sysIdTestTab
        .add(
            "Shooter: Dynamic Reverse",
            RobotConfig.shooter.sysIdDynamic(SysIdRoutine.Direction.kReverse))
        .withPosition(colIndex, rowIndex++)
        .withSize(2, 1);

    /*
    colIndex++;
    rowIndex = 0;
    sysIdTestTab.add("Drive: Drive Motors", RobotConfig.drive.sysIdDriveMotorCommand()).withPosition(colIndex, rowIndex++).withSize(2, 1);
    sysIdTestTab.add("Drive: Angle Motors", RobotConfig.drive.sysIdAngleMotorCommand()).withPosition(colIndex, rowIndex++).withSize(2, 1);
    */
  }
}
