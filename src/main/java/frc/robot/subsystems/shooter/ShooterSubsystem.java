package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.config.RobotConfig.ShooterConstants;
import java.util.ArrayList;
import java.util.List;
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

  // Mechanism2d display of a Shooter
  private List<MechanismLigament2d> shooter2d = new ArrayList<MechanismLigament2d>();
  private int currentSimAngle = 0;

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

    // Create 2D simulated display of a Shooter
    Mechanism2d mech2d = new Mechanism2d(60, 60);
    MechanismRoot2d intakePivot2d = mech2d.getRoot("Shooter Pivot", 30, 30);

    shooter2d.add(
        intakePivot2d.append(
            new MechanismLigament2d("Wheel Spoke A", 15, 0, 12, new Color8Bit(Color.kGray))));
    shooter2d.add(
        intakePivot2d.append(
            new MechanismLigament2d("Wheel Spoke B", 15, 90, 12, new Color8Bit(Color.kBlue))));
    shooter2d.add(
        intakePivot2d.append(
            new MechanismLigament2d("Wheel Spoke C", 15, 180, 12, new Color8Bit(Color.kGray))));
    shooter2d.add(
        intakePivot2d.append(
            new MechanismLigament2d("Wheel Spoke D", 15, 270, 12, new Color8Bit(Color.kBlue))));

    SmartDashboard.putData("Shooter Simulation", mech2d);
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

    if (velocityRPM != 0) {
      if (velocityRPM < 0) {
        currentSimAngle += (velocityRPM / 6000) * 45;
      } else if (velocityRPM > 0) {
        currentSimAngle -= (velocityRPM / 6000) * 45;
      }

      int angleOffset = 0;
      for (MechanismLigament2d shooter : shooter2d) {
        shooter.setAngle(angleOffset + currentSimAngle);
        angleOffset += 90;
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
