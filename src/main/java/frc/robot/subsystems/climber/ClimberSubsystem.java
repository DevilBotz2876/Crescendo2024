package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.RobotConfig.ClimberConstants;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class ClimberSubsystem extends SubsystemBase implements Climber {
  private class ClimberInstance {
    private final ClimberIO io;
    private final ClimberIOInputsAutoLogged inputs;
    private final String name;
    private double voltage = 0.0;
    private boolean extend = false;
    private boolean autoZeroMode = false;
    private boolean enableLimits = true;

    private MechanismLigament2d climber2d = null;

    ClimberInstance(ClimberIO io, String name) {
      this.io = io;
      this.name = name;
      inputs = new ClimberIOInputsAutoLogged();
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

      if ((null != climber2d) && (voltage != 0)) {
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

    public void add2dSim(MechanismRoot2d root2d) {
      climber2d =
          root2d.append(new MechanismLigament2d(name, 10, 90, 6, new Color8Bit(Color.kSilver)));
    }
  }

  private final ClimberInstance left, right;
  private final List<ClimberInstance> climbers = new ArrayList<ClimberInstance>();
  private boolean extend = false;

  public ClimberSubsystem(ClimberIO left, ClimberIO right) {
    this.left = new ClimberInstance(left, "Climber Left");
    climbers.add(this.left);

    this.right = new ClimberInstance(right, "Climber Right");
    climbers.add(this.right);
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

  public void PrepareArmForMatch() {}

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

  private void autoZeroMode(boolean enable) {
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
    return new SequentialCommandGroup(
        new InstantCommand(() -> extend(), this),
        Commands.waitUntil(() -> left.isAtMaxLimit() && right.isAtMaxLimit()));
  }

  @Override
  public Command getRetractCommand() {
    return new SequentialCommandGroup(
        new InstantCommand(() -> retract(), this),
        Commands.waitUntil(() -> left.isAtMinLimit() && right.isAtMinLimit()));
  }

  @Override
  public Command getAutoZeroCommand() {
    return new SequentialCommandGroup(
            new InstantCommand(
                () -> {
                  System.out.println("Auto Zero: Disable Limits");
                  enableLimits(false);
                  System.out.println("Auto Zero: Extend Climber");
                  runVoltage(ClimberConstants.autoZeroVoltage);
                  System.out.println("Auto Zero: Wait For Extend");
                },
                this),
            Commands.waitSeconds(ClimberConstants.autoZeroExtendTimeInSeconds),
            new InstantCommand(
                () -> {
                  System.out.println("Auto Zero: Enable Limits");
                  enableLimits(true);
                  System.out.println("Auto Zero: Enable Auto Zero Mode");
                  autoZeroMode(true);
                  System.out.println("Auto Zero: Retract Climber");
                  runVoltage(-ClimberConstants.autoZeroVoltage);
                  System.out.println("Auto Zero: Wait until done or timeout");
                },
                this),
            Commands.waitSeconds(ClimberConstants.autoZeroMaxRetractTimeInSeconds)
                .until(() -> ((left.autoZeroMode == false) && (right.autoZeroMode == false))))
        .finallyDo(
            () -> {
              System.out.println("Auto Zero: Disable Auto Zero Mode");
              autoZeroMode(false);
              System.out.println("Auto Zero: Turn Off Climber");
              runVoltage(0);
            });
  }

  @Override
  public void overridePosition(double leftpos, double rightPos) {
    runVoltage(0);
    left.io.setPosition(leftpos);
    right.io.setPosition(rightPos);
  }

  @Override
  public void add2dSim(Mechanism2d mech2d) {
    left.add2dSim(mech2d.getRoot("Left Climber Pivot", 5, 10));
    right.add2dSim(mech2d.getRoot("Right Climber Pivot", 55, 10));
  }
}
