package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
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

  // Values from Carter's Shooter SysId Run on Sherman 2024-02-07
  double ffKs = 0.08134;
  double ffKv = 0.019999;
  double ffKa = 0.0054252;
  double ffKsBottom = 0.058262;
  double ffKvBottom = 0.019495;
  double ffKaBottom = 0.0048198;

  double pidKp = 0.0010514;
  double pidKi = 0.0;
  double pidKd = 0.0;
  double pidKpBottom = 0.0001581;
  double pidKiBottom = 0.0;
  double pidKdBottom = 0.0;

  public ShooterSubsystem(ShooterIO io) {
    this.io = io;
    // TODO: These are sample values.  Need to run sysid on shooter and get real values.
    useSoftwarePid = !io.supportsHardwarePid();
    if (useSoftwarePid) {
      feedforward = new SimpleMotorFeedforward(ffKs, ffKv, ffKa);
      feedforwardBottom = new SimpleMotorFeedforward(ffKsBottom, ffKvBottom, ffKaBottom);
      pid = new PIDController(pidKp, pidKi, pidKd);
      pidBottom = new PIDController(pidKpBottom, pidKiBottom, pidKdBottom);
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
