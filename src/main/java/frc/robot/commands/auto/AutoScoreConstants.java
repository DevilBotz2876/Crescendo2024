package frc.robot.commands.auto;

public class AutoScoreConstants {
  public double robotYawInDegrees;
  public double armAngleInDegrees;
  public double shooterVelocityInRPMs;

  public AutoScoreConstants(
      double robotYawInDegrees, double armAngleInDegrees, double shooterVelocityInRPMs) {
    this.robotYawInDegrees = robotYawInDegrees;
    this.armAngleInDegrees = armAngleInDegrees;
    this.shooterVelocityInRPMs = shooterVelocityInRPMs;
  }
}
