package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.config.RobotConfig.ShooterConstants;
import java.util.Map;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class ShooterSubsystem extends SubsystemBase implements Shooter {
  ShooterIO io;
  ShooterIO ioBottom = null;
  private final SimpleMotorFeedforward feedforward;
  private final SimpleMotorFeedforward feedforwardBottom;
  private final PIDController pid;
  private final PIDController pidBottom;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
  private final SysIdRoutine sysId;
  private final ShooterIOInputsAutoLogged inputsBottom = new ShooterIOInputsAutoLogged();
  @AutoLogOutput private double voltage;
  @AutoLogOutput private double velocityRPM;
  @AutoLogOutput private double targetVelocityRadPerSec;
  boolean useSoftwarePid = false;
  boolean softwarePidEnabled = false;

  public ShooterSubsystem(ShooterIO io) {
    this.io = io;
    // TODO: These are sample values.  Need to run sysid on shooter and get real values.
    useSoftwarePid = !io.supportsHardwarePid();
    if (useSoftwarePid) {
      feedforward =
          new SimpleMotorFeedforward(
              ShooterConstants.ffKs, ShooterConstants.ffKv, ShooterConstants.ffKa);
      feedforwardBottom =
          new SimpleMotorFeedforward(
              ShooterConstants.ffKsBottom,
              ShooterConstants.ffKvBottom,
              ShooterConstants.ffKaBottom);
      pid =
          new PIDController(ShooterConstants.pidKp, ShooterConstants.pidKi, ShooterConstants.pidKd);
      pidBottom =
          new PIDController(
              ShooterConstants.pidKpBottom,
              ShooterConstants.pidKiBottom,
              ShooterConstants.pidKdBottom);
    } else {
      feedforward = null;
      feedforwardBottom = null;
      pid = null;
      pidBottom = null;
    }
    voltage = 0;
    velocityRPM = 0.0;

    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Shooter/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism((voltage) -> runVoltage(voltage.in(Volts)), null, this));

    // create shooter tab on ShuffleBoard
    ShuffleboardTab tab = Shuffleboard.getTab("Assist");
    // Create volt entry under Shooter tab as a number sider with min = -1 and max = 1
    tab.add("Shooter Velocity", ShooterConstants.velocityInRPMs)
        .withWidget(BuiltInWidgets.kTextView)
        .withProperties(Map.of("min", 0, "max", 6000));
  }

  public ShooterSubsystem(ShooterIO ioTop, ShooterIO ioBottom) {
    this(ioTop);
    this.ioBottom = ioBottom;
  }

  @Override
  // Sets the voltage to volts. the volts value is -12 to 12
  public void runVoltage(double volts) {
    voltage = volts;
    targetVelocityRadPerSec = 0;
    io.setVoltage(voltage);
    if (ioBottom != null) {
      ioBottom.setVoltage(voltage);
    }
  }

  @Override
  public void runVelocity(double velocityRPM) {
    voltage = 0;
    this.velocityRPM = velocityRPM;
    targetVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM);

    if (useSoftwarePid) {
      softwarePidEnabled = true;
    } else {
      io.setVelocity(targetVelocityRadPerSec, feedforward.calculate(targetVelocityRadPerSec));
      if (ioBottom != null) {
        ioBottom.setVelocity(
            targetVelocityRadPerSec, feedforwardBottom.calculate(targetVelocityRadPerSec));
      }
    }
  }

  @Override
  public double getVoltage() {
    return inputs.appliedVolts;
  }

  @Override
  public double getCurrentSpeed() {
    return inputs.velocityRadPerSec;
  }

  @Override
  public void periodic() {
    // Updates the inputs
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);

    if (ioBottom != null) {
      ioBottom.updateInputs(inputsBottom);
      Logger.processInputs("Shooter Bottom", inputsBottom);
    }

    if (softwarePidEnabled) {
      io.setVoltage(
          feedforward.calculate(targetVelocityRadPerSec)
              + pid.calculate(inputs.velocityRadPerSec, targetVelocityRadPerSec));
      if (ioBottom != null) {
        ioBottom.setVoltage(
            feedforwardBottom.calculate(targetVelocityRadPerSec)
                + pidBottom.calculate(inputsBottom.velocityRadPerSec, targetVelocityRadPerSec));
      }
    }
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysId.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysId.dynamic(direction);
  }
}
