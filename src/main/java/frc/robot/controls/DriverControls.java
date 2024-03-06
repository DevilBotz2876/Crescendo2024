package frc.robot.controls;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.assist.EjectPiece;
import frc.robot.commands.assist.PrepareForIntake;
import frc.robot.commands.assist.ScorePiece;
import frc.robot.commands.climber.ClimberCommand;
import frc.robot.commands.drive.DriveCommand;
import frc.robot.commands.shooter.SetShooterVelocity;
import frc.robot.config.RobotConfig;
import frc.robot.util.RobotState;
import frc.robot.util.RobotState.TargetMode;
import java.util.Optional;

public class DriverControls {
  public static void setupDriverControls(CommandXboxController mainController) {
    /* Drive Controls */
    /* Controller 0:
     *     Left Stick Up/Down = *Field Oriented* Drive Away/Towards Driver Station
     *     Left Stick Left/Right = *Field Oriented* Drive Left/Right relative to Driver Station
     *     Right Bumper = Toggles Auto Orient/Prepare For Score
     *     Right Stick Left/Right
     *         Auto Orient Off: Rotate CCW/CW
     *         Auto Orient On: Vision based rotation/arm angle
     */
    RobotConfig.drive.setFieldOrientedDrive(true);

    RobotConfig.drive.setDefaultCommand(
        new DriveCommand(
            RobotConfig.drive,
            () -> MathUtil.applyDeadband(-mainController.getLeftY(), 0.05),
            () -> MathUtil.applyDeadband(-mainController.getLeftX(), 0.05),
            () -> MathUtil.applyDeadband(-mainController.getRightX(), 0.05)));

    /* Climber Controls */
    /*     D-Pad Up = Climber Up
     *     D-Pad Down = Climber Down
     */
    mainController.pov(0).onTrue(new ClimberCommand(RobotConfig.climber, true));
    mainController.pov(180).onTrue(new ClimberCommand(RobotConfig.climber, false));

    /* Driver Assist Controls */
    /*     Left Trigger = "Prepare for Intake"
     *     Right Trigger = "Score Piece"
     *     Left Bumper = "Eject Piece"
     *     Right Bumper = Toggle ("Protect Arm" <--> "Prepare for Score")
     */
    mainController.leftTrigger().onTrue(new PrepareForIntake(RobotConfig.arm, RobotConfig.intake));
    mainController.rightTrigger().onTrue(new ScorePiece(RobotConfig.intake, RobotConfig.shooter));
    mainController.leftBumper().onTrue(new EjectPiece(RobotConfig.intake, RobotConfig.arm));

    /* TODO: Make this is a toggle, not just while right bumper is pressed */
    mainController
        .rightBumper()
        .whileTrue(
            new ParallelCommandGroup(
                new DriveCommand(
                    RobotConfig.drive,
                    () -> MathUtil.applyDeadband(-mainController.getLeftY(), 0.05),
                    () -> MathUtil.applyDeadband(-mainController.getLeftX(), 0.05),
                    () -> MathUtil.applyDeadband(-mainController.getRightX(), 0.05),
                    () -> RobotConfig.vision.getYawToAprilTag(RobotState.getActiveTargetId())),
                new SetShooterVelocity(RobotConfig.shooter, () -> RobotState.getShooterVelocity()),
                /* TODO: Use ArmToPositionTP instead of setting arm angle directly */
                new InstantCommand(
                        () -> {
                          Optional<Double> distanceToTarget =
                              RobotConfig.vision.getDistanceToAprilTag(
                                  RobotState.getActiveTargetId());
                          if (distanceToTarget.isPresent()) {
                            Optional<Double> armAngle =
                                RobotConfig.instance.getArmAngleFromDistance(
                                    distanceToTarget.get());
                            if (armAngle.isPresent()) {
                              RobotConfig.arm.setAngle((armAngle.get()));
                            }
                          }
                        })
                    .repeatedly()));

    /* Target Selection Controls */
    /*     A Button = Amp Mode
     *     B Button = Speaker Mode
     */
    mainController.a().onTrue(new InstantCommand(() -> RobotState.setTargetMode(TargetMode.AMP)));
    mainController
        .b()
        .onTrue(new InstantCommand(() -> RobotState.setTargetMode(TargetMode.SPEAKER)));
  }
}
