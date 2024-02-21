package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.RobotConfig.ClimberConstants;
import java.util.Map;
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

    ShuffleboardTab assistTab = Shuffleboard.getTab("Assist");
    // Create volt entry under Shooter tab as a number sider with min = -1 and max = 1
    assistTab
        .add("Climber Volts", ClimberConstants.maxSpeedInVolts)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", 0, "max", 12));
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

  @Override
  public void setVoltage(double volts) {
    voltage = volts;
  }

  private boolean leftAtLimits() {
    return bExtend
        ? (inputsLeft.positionRadians >= ClimberConstants.maxPositionInRadians)
        : (inputsLeft.positionRadians <= ClimberConstants.minPositionInRadians);
  }

  private boolean rightAtLimits() {
    return bExtend
        ? (inputsRight.positionRadians >= ClimberConstants.maxPositionInRadians)
        : (inputsRight.positionRadians <= ClimberConstants.minPositionInRadians);
  }
}
