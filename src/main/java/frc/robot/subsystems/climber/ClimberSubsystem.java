package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.RobotConfig.ClimberConstants;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class ClimberSubsystem extends SubsystemBase implements Climber {
  private class ClimberInstance {
    private final ClimberIO io;
    private final ClimberIOInputsAutoLogged inputs;
    @AutoLogOutput private final String name;
    private double voltage = 0.0;
    @AutoLogOutput private boolean atLimit = false;
    private boolean extend = false;
    private boolean autoZeroMode = false;
    private boolean enableLimits = true;

    private final MechanismLigament2d climber2d;

    ClimberInstance(ClimberIO io, String name, MechanismRoot2d root2d) {
      this.io = io;
      this.name = name;
      inputs = new ClimberIOInputsAutoLogged();

      climber2d =
          root2d.append(new MechanismLigament2d(name, 10, 90, 6, new Color8Bit(Color.kSilver)));
    }

    void periodic() {
      // Updates the inputs
      io.updateInputs(inputs);
      Logger.processInputs(name, inputs);

      if (atLimits()) {
        io.setVoltage(0);
      } else {
        io.setVoltage(voltage);
      }

      if (voltage != 0) {
        climber2d.setLength(
            10 + 30 * (inputs.positionRadians / ClimberConstants.maxPositionInRadians));
      }
    }

    public void runVoltage(double volts) {
      if (volts > 0) {
        extend = true;
      } else {
        extend = false;
      }
      voltage = volts;
    }

    private boolean isAtMaxLimit() {
      return (inputs.positionRadians >= ClimberConstants.maxPositionInRadians);
    }

    private boolean isAtMinLimit() {
      return (inputs.positionRadians <= ClimberConstants.minPositionInRadians);
    }

    private boolean atLimits() {
      if (enableLimits == false) return false;

      if (autoZeroMode) {
        if (extend) {
          /* If we are in autozero mode, don't let the climber move up */
          return true;
        } else {
          if ((inputs.current > ClimberConstants.autoZeroMaxCurrent)
              && (Math.abs(inputs.velocityRadiansPerSecond)
                  < ClimberConstants.autoZeroMinVelocity)) {
            runVoltage(0);
            io.setPosition(ClimberConstants.autoZeroOffset);
            autoZeroMode = false;
            return true;
          } else {
            return false;
          }
        }
      } else {
        boolean atLimit = extend ? isAtMaxLimit() : isAtMinLimit();
        ;
        if (atLimit) {
          runVoltage(0);
        }
        return atLimit;
      }
    }

    public void resetPosition() {
      io.setPosition(0);
    }

    public void autoZeroMode(boolean enable) {
      autoZeroMode = enable;
    }

    public void enableLimits(boolean enable) {
      enableLimits = enable;
    }
  }

  private final ClimberInstance left, right;
  private final List<ClimberInstance> climbers = new ArrayList<ClimberInstance>();
  private boolean extend = false;

  public ClimberSubsystem(ClimberIO left, ClimberIO right) {
    // Create 2D simulated display of a Climber
    Mechanism2d mech2d = new Mechanism2d(60, 60);

    this.left =
        new ClimberInstance(left, "Climber Left", mech2d.getRoot("Left Climber Pivot", 10, 10));
    climbers.add(this.left);

    this.right =
        new ClimberInstance(right, "Climber Right", mech2d.getRoot("Right ClimberPivot", 50, 10));
    climbers.add(this.right);

    SmartDashboard.putData("Climber Simulation", mech2d);
  }

  @Override
  public void periodic() {
    for (ClimberInstance climber : climbers) {
      climber.periodic();
    }
  }

  /** Extends climber arms min limit */
  @Override
  public void extend() {
    extend = true;
    runVoltage(ClimberConstants.defaultSpeedInVolts);
  }

  /** Retracts climber to min limit */
  @Override
  public void retract() {
    extend = false;
    runVoltage(-ClimberConstants.defaultSpeedInVolts);
  }

  @Override
  public boolean isExtending() {
    return extend;
  }

  @Override
  public void runVoltage(double volts) {
    runVoltageLeft(volts);
    runVoltageRight(volts);
  }

  @Override
  public void runVoltageLeft(double volts) {
    left.runVoltage(volts);
  }

  @Override
  public void runVoltageRight(double volts) {
    right.runVoltage(volts);
  }

  @Override
  public void resetPosition() {
    for (ClimberInstance climber : climbers) {
      climber.resetPosition();
    }
  }

  @Override
  public void autoZeroMode(boolean enable) {
    for (ClimberInstance climber : climbers) {
      climber.autoZeroMode(enable);
    }
  }

  @Override
  public void enableLimits(boolean enable) {
    for (ClimberInstance climber : climbers) {
      climber.enableLimits(enable);
    }
  }

  @Override
  public double getCurrentPositionLeft() {
    return left.inputs.positionRadians;
  }

  @Override
  public double getCurrentPositionRight() {
    return right.inputs.positionRadians;
  }

  @Override
  public boolean isAtMaxLimitLeft() {
    return left.isAtMaxLimit();
  }

  @Override
  public boolean isAtMinLimitLeft() {
    return left.isAtMinLimit();
  }

  @Override
  public boolean isAtMaxLimitRight() {
    return right.isAtMaxLimit();
  }

  @Override
  public boolean isAtMinLimitRight() {
    return right.isAtMinLimit();
  }

  @Override
  public Command getExtendCommand() {
    return runOnce(() -> extend());
  }

  @Override
  public Command getRetractCommand() {
    return runOnce(() -> retract());
  }
}
