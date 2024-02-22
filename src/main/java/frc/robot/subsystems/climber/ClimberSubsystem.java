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
  @AutoLogOutput private double leftVoltage = 0.0;
  @AutoLogOutput private double rightVoltage = 0.0;
  @AutoLogOutput private boolean bExtendLeft = false;
  @AutoLogOutput private boolean bExtendRight = false;
  @AutoLogOutput private boolean leftAtLimit = false;
  @AutoLogOutput private boolean rightAtLimit = false;
  @AutoLogOutput private boolean enableLimits = true;

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

    if (!leftAtLimits()) left.setVoltage(leftVoltage);
    else left.setVoltage(0);

    if (!rightAtLimits()) right.setVoltage(rightVoltage);
    else right.setVoltage(0);
  }

  /** Extends climber arms min limit */
  public void extend() {
    runVoltage(ClimberConstants.defaultSpeedInVolts);
  }

  /** Retracts climber to min limit */
  public void retract() {
    runVoltage(-ClimberConstants.defaultSpeedInVolts);
  }

  @Override
  public void runVoltage(double volts) {
    runVoltageLeft(volts);
    runVoltageRight(volts);
  }

  @Override
  public void runVoltageLeft(double volts) {
    if (volts > 0) {
      bExtendLeft = true;
    } else {
      bExtendLeft = false;
    }
    leftVoltage = volts;
  }

  @Override
  public void runVoltageRight(double volts) {
    if (volts > 0) {
      bExtendRight = true;
    } else {
      bExtendRight = false;
    }
    rightVoltage = volts;
  }

  private boolean leftAtLimits() {
    leftAtLimit =
        bExtendLeft
            ? (inputsLeft.positionRadians >= ClimberConstants.maxPositionInRadians)
            : (inputsLeft.positionRadians <= ClimberConstants.minPositionInRadians);
    return enableLimits ? leftAtLimit : false;
  }

  private boolean rightAtLimits() {
    rightAtLimit =
        bExtendRight
            ? (inputsRight.positionRadians >= ClimberConstants.maxPositionInRadians)
            : (inputsRight.positionRadians <= ClimberConstants.minPositionInRadians);
    return enableLimits ? rightAtLimit : false;
  }

  public void resetPosition() {
    left.resetPosition();
    right.resetPosition();
  }

  public void enableLimits(boolean enable) {
    enableLimits = enable;
  }
}
