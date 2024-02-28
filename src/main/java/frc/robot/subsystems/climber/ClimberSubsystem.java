package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
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
  @AutoLogOutput private boolean autoZeroModeLeft = false;
  @AutoLogOutput private boolean autoZeroModeRight = false;
  @AutoLogOutput private boolean enableLimits = true;

  // Create a Mechanism2d display of a Climber
  private final Mechanism2d mech2d = new Mechanism2d(60, 60);
  private final MechanismRoot2d leftClimberPivot2d = mech2d.getRoot("LeftClimberPivot", 10, 10);
  private final MechanismRoot2d rightClimberPivot2d = mech2d.getRoot("RightClimberPivot", 50, 10);
  private final MechanismLigament2d leftClimber2d =
      leftClimberPivot2d.append(
          new MechanismLigament2d("LeftClimber", 10, 90, 6, new Color8Bit(Color.kSilver)));
  private final MechanismLigament2d rightClimber2d =
      rightClimberPivot2d.append(
          new MechanismLigament2d("RightClimber", 10, 90, 6, new Color8Bit(Color.kSilver)));

  public ClimberSubsystem(ClimberIO left, ClimberIO right) {
    this.left = left;
    this.right = right;

    SmartDashboard.putData("Climber Simulation", mech2d);
  }

  @Override
  public void periodic() {
    // Updates the inputs
    left.updateInputs(inputsLeft);
    right.updateInputs(inputsRight);
    Logger.processInputs("Climber Left", inputsLeft);
    Logger.processInputs("Climber Right", inputsRight);

    if (leftAtLimits()) {
      left.setVoltage(0);
    } else {
      left.setVoltage(leftVoltage);
    }

    if (rightAtLimits()) {
      right.setVoltage(0);
    } else {
      right.setVoltage(rightVoltage);
    }

    //    if (leftVoltage != 0)
    {
      leftClimber2d.setLength(
          10 + 30 * (inputsLeft.positionRadians / ClimberConstants.maxPositionInRadians));
    }

    //    if (rightVoltage != 0)
    {
      rightClimber2d.setLength(
          10 + 30 * (inputsRight.positionRadians / ClimberConstants.maxPositionInRadians));
    }
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
    if (enableLimits == false) return false;

    if (autoZeroModeLeft) {
      if (bExtendLeft) {
        /* If we are in autozero mode, don't let the climber move up */
        return true;
      } else {
        if ((inputsLeft.current > ClimberConstants.autoZeroMaxCurrent)
            && (Math.abs(inputsLeft.appliedVolts - leftVoltage)
                > ClimberConstants.autoZeroMaxVoltageDelta)) {
          left.setPosition(ClimberConstants.autoZeroOffset);
          autoZeroModeLeft = false;
          return true;
        } else {
          return false;
        }
      }
    } else {
      return bExtendLeft
          ? (inputsLeft.positionRadians >= ClimberConstants.maxPositionInRadians)
          : (inputsLeft.positionRadians <= ClimberConstants.minPositionInRadians);
    }
  }

  private boolean rightAtLimits() {
    if (enableLimits == false) return false;

    if (autoZeroModeRight) {
      if (bExtendRight) {
        /* If we are in autozero mode, don't let the climber move up */
        return true;
      } else {
        if ((inputsRight.current > ClimberConstants.autoZeroMaxCurrent) // Current is high
            && (Math.abs(
                    inputsRight.appliedVolts
                        - rightVoltage) // There's a large enough difference between applied volts
                // and the requested voltage
                > ClimberConstants.autoZeroMaxVoltageDelta)) {
          right.setPosition(ClimberConstants.autoZeroOffset);
          autoZeroModeRight = false;
          return true;
        } else {
          return false;
        }
      }
    } else {
      return bExtendRight
          ? (inputsRight.positionRadians >= ClimberConstants.maxPositionInRadians)
          : (inputsRight.positionRadians <= ClimberConstants.minPositionInRadians);
    }
  }

  @Override
  public void resetPosition() {
    left.setPosition(0);
    right.setPosition(0);
  }

  @Override
  public void autoZeroMode(boolean enable) {
    autoZeroModeLeft = enable;
    autoZeroModeRight = enable;
  }

  @Override
  public void enableLimits(boolean enable) {
    enableLimits = enable;
  }
}
