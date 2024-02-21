package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.RobotConfig.ClimberConstants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class ClimberSubsystem extends SubsystemBase implements Climber {
  private final ClimberIO left;
  private final ClimberIO right;
  private final ClimberIOInputsAutoLogged inputsLeft = new ClimberIOInputsAutoLogged();
  private final ClimberIOInputsAutoLogged inputsRight = new ClimberIOInputsAutoLogged();
  private double voltage = 0.0;
  @AutoLogOutput private boolean bExtend = false;

  public ClimberSubsystem(ClimberIO left, ClimberIO right) {
    this.left = left;
    this.right = right;
  }

  @Override
  public void periodic() {
    // Updates the inputs
    left.updateInputs(inputsLeft);
    right.updateInputs(inputsRight);
    Logger.processInputs("Climber Left", inputsLeft);
    Logger.processInputs("Climber Right", inputsRight);

    if (!leftAtLimits()) left.setVoltage(voltage);
    else left.setVoltage(0);

    if (!rightAtLimits()) right.setVoltage(voltage);
    else right.setVoltage(0);
  }

  /** Extends climber arms min limit */
  public void extend() {
    bExtend = true;
    voltage = ClimberConstants.maxSpeedInVolts;
  }

  /** Retracts climber to min limit */
  public void retract() {
    bExtend = false;
    voltage = -ClimberConstants.maxSpeedInVolts;
  }

  private boolean leftAtLimits() {
    return bExtend
        ? (inputsLeft.positionRadians >= ClimberConstants.maxPositionInMeters)
        : (inputsLeft.positionRadians <= ClimberConstants.minPositionInMeters);
  }

  private boolean rightAtLimits() {
    return bExtend
        ? (inputsRight.positionRadians >= ClimberConstants.maxPositionInMeters)
        : (inputsRight.positionRadians <= ClimberConstants.minPositionInMeters);
  }
}
