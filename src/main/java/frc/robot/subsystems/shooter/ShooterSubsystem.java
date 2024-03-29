package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.config.RobotConfig.ShooterConstants;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class ShooterSubsystem extends ProfiledPIDSubsystem implements Shooter {
  ShooterIO io;
  private final SimpleMotorFeedforward feedforward;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
  boolean useSoftwarePidVelocityControl = true;

  private final SysIdRoutine sysId;
  @AutoLogOutput private double targetVoltage;
  @AutoLogOutput private double targetVelocityRPM;
  @AutoLogOutput private double targetVelocityRadPerSec;
  @AutoLogOutput private double currentVelocitySetpointRPM;

  // Mechanism2d display of a Shooter
  private List<MechanismLigament2d> shooter2d = new ArrayList<MechanismLigament2d>();
  private int currentSimAngle = 0;

  public ShooterSubsystem(ShooterIO io) {
    super(
        new ProfiledPIDController(
            ShooterConstants.pidKp,
            ShooterConstants.pidKi,
            ShooterConstants.pidKd,
            new TrapezoidProfile.Constraints(
                Units.rotationsPerMinuteToRadiansPerSecond(ShooterConstants.maxVelocityInRPM),
                Units.rotationsPerMinuteToRadiansPerSecond(
                    ShooterConstants.maxAccelerationInRPMSquared))));

    this.io = io;
    useSoftwarePidVelocityControl = !io.supportsHardwarePid();

    //    if (false == useSoftwareVelocityControl)
    //    {
    //      io.setPID(
    //        ShooterConstants.pidKp,
    //        ShooterConstants.pidKi,
    //        ShooterConstants.pidKd);
    //    }

    feedforward =
        new SimpleMotorFeedforward(
            ShooterConstants.ffKs, ShooterConstants.ffKv, ShooterConstants.ffKa);

    targetVoltage = 0;
    targetVelocityRPM = 0.0;

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
  }

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    double ff = feedforward.calculate(setpoint.position);

    if (useSoftwarePidVelocityControl) {
      // Use feedforward +  SW velocity PID
      io.setVoltage(output + ff);
    } else {
      // Use feedforward +  HW velocity PID (ignore SW PID)
      io.setVelocity(setpoint.position, ff);
    }

    currentVelocitySetpointRPM = Units.radiansPerSecondToRotationsPerMinute(setpoint.position);
    //    System.out.println(setpoint.position);
  }

  @Override
  public double getMeasurement() {
    return inputs.velocityRadPerSec;
  }

  @Override
  // Sets the voltage to volts. the volts value is -12 to 12
  public void runVoltage(double volts) {
    targetVoltage = volts;
    targetVelocityRadPerSec = 0;
    this.targetVelocityRPM = ShooterConstants.maxVelocityInRPM * (volts / 12.0);
    disable(); // disable PID control
    io.setVoltage(targetVoltage);
  }

  @Override
  public void runVelocity(double velocityRPM) {
    velocityRPM = MathUtil.clamp(velocityRPM, 0, ShooterConstants.maxVelocityInRPM);
    targetVoltage = -1;
    this.targetVelocityRPM = velocityRPM;
    targetVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM);
    setGoal(targetVelocityRadPerSec);
    enable();
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
    super.periodic();
    // Updates the inputs
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);

    currentSimAngle -=
        (inputs.velocityRadPerSec
                / Units.rotationsPerMinuteToRadiansPerSecond(ShooterConstants.maxVelocityInRPM))
            * 45;

    int angleOffset = 0;
    for (MechanismLigament2d shooter : shooter2d) {
      shooter.setAngle(angleOffset + currentSimAngle);
      angleOffset += 90;
    }
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysId.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysId.dynamic(direction);
  }

  public Command getTurnOffCommand() {
    return runOnce(() -> turnOff());
  }

  @Override
  public void add2dSim(Mechanism2d mech2d) {
    // Create 2D simulated display of a Shooter
    MechanismRoot2d intakePivot2d = mech2d.getRoot("Shooter Pivot", 15, 50);

    shooter2d.add(
        intakePivot2d.append(
            new MechanismLigament2d("Wheel Spoke A", 5, 0, 12, new Color8Bit(Color.kGray))));
    shooter2d.add(
        intakePivot2d.append(
            new MechanismLigament2d("Wheel Spoke B", 5, 90, 12, new Color8Bit(Color.kBlue))));
    shooter2d.add(
        intakePivot2d.append(
            new MechanismLigament2d("Wheel Spoke C", 5, 180, 12, new Color8Bit(Color.kGray))));
    shooter2d.add(
        intakePivot2d.append(
            new MechanismLigament2d("Wheel Spoke D", 5, 270, 12, new Color8Bit(Color.kBlue))));
  }

  @Override
  public boolean isAtSetpoint() {
    return (Math.abs(currentVelocitySetpointRPM - targetVelocityRPM)
        < ShooterConstants.pidVelocityErrorInRPM);
  }
}
