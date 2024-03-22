package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.config.RobotConfig.ArmConstants;
import frc.robot.util.DevilBotState;
import frc.robot.util.DevilBotState.State;
import frc.robot.util.TrapezoidProfileSubsystem2876;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class ArmSubsystem extends TrapezoidProfileSubsystem2876 implements Arm {
  private final ArmIO io;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
  private ArmFeedforward feedforward =
      new ArmFeedforward(
          ArmConstants.ffKs, ArmConstants.ffKg, ArmConstants.ffKv, ArmConstants.ffKa);

  private final SysIdRoutine sysId;
  private final double positionDegreeMax = ArmConstants.maxAngleInDegrees;
  private final double positionDegreeMin = ArmConstants.minAngleInDegrees;
  @AutoLogOutput private double targetVoltage;
  @AutoLogOutput private double targetDegrees;
  @AutoLogOutput private double targetRelativeDegrees;

  // Create a Mechanism2d display of an Arm with a fixed ArmTower and moving Arm.
  private final double armAngle2dOffset = 0;
  private MechanismLigament2d arm2d = null;

  public ArmSubsystem(ArmIO io) {
    super(
        new TrapezoidProfile.Constraints(
            ArmConstants.maxVelocityInDegreesPerSecond,
            ArmConstants.maxAccelerationInDegreesPerSecondSquared));

    this.io = io;

    // Configure SysId based on the AdvantageKit example
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Arm/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism((voltage) -> runVoltage(voltage.in(Volts)), null, this));
  }

  @Override
  public void useState(TrapezoidProfile.State setpoint) {
    double ff = feedforward.calculate(setpoint.position, setpoint.velocity);

    // Use feedforward +  HW velocity PID (ignore SW PID)
    io.setPosition(setpoint.position, ff);

    //    System.out.println(setpoint.position + ":" + output);
  }

  @Override
  public TrapezoidProfile.State getMeasurement() {
    return new TrapezoidProfile.State(getRelativeAngle(), getVelocity());
  }

  @Override
  public double getAngle() {
    return inputs.positionDegrees;
  }

  @Override
  public double getVelocity() {
    return inputs.velocityDegreesPerSecond;
  }

  @Override
  public double getRelativeAngle() {
    return inputs.relativePositionDegrees;
  }

  @Override
  public double getTargetAngle() {
    return targetDegrees;
  }

  private void syncRelativeSetpointToAbsoluteSetpoint() {
    // To account for differences in absolute encoder and relative encoder readings cause by
    // backlash and other arm physics,
    // we calculate the difference in current vs target absolute encoder value and then calculate
    // the corrsponding relative
    // angle
    double deltaDegrees = this.targetDegrees - getAngle();
    this.targetRelativeDegrees = getRelativeAngle() + deltaDegrees;

    setTargetAngle(this.targetRelativeDegrees);
    enable();
  }

  private void setTargetAngle(double degrees) {
    setGoal(degrees);
    //    setSetpoint(degrees);
  }

  // sets of the angle of the arm
  @Override
  public void setAngle(double degrees) {
    degrees =
        MathUtil.clamp(degrees, ArmConstants.minAngleInDegrees, ArmConstants.maxAngleInDegrees);

    // Don't try to set position if absolute encoder is broken/missing.
    if (isAbsoluteEncoderConnected() == false) {
      return;
    }
    if (isAbsoluteEncoderReadingValid() == false) {
      return;
    }

    this.targetDegrees = degrees;

    syncRelativeSetpointToAbsoluteSetpoint();
  }

  @Override
  public boolean isAbsoluteEncoderConnected() {
    return io.isAbsoluteEncoderConnected();
  }

  public boolean isAbsoluteEncoderReadingValid() {
    if (getAngle() > ArmConstants.minAngleInDegrees - 10
        && getAngle() < ArmConstants.maxAngleInDegrees + 10) {
      return true;
    }
    return false;
  }

  // Sets the voltage to volts. the volts value is -12 to 12
  public void runVoltage(double volts) {
    targetVoltage = voltageSafety(volts);
    disable(); // disable PID control
    io.setVoltage(targetVoltage);
  }

  protected double voltageSafety(double voltage) {
    if (isLimitReached(voltage)) return 0.0;
    else return voltage;
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysId.quasistatic(direction);
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysId.dynamic(direction);
  }

  @Override
  public void periodic() {
    super.periodic();
    // Updates the inputs
    io.updateInputs(inputs);
    Logger.processInputs("Arm", inputs);

    if (DevilBotState.getState() == State.DISABLED && io.isAbsoluteEncoderConnected()) {
      io.resetRelativeEncoder(getAngle());
    }

    if (isLimitHigh() && inputs.appliedVolts > 0) {
      io.setVoltage(0);
    }
    if (isLimitLow() && inputs.appliedVolts < 0) {
      io.setVoltage(0);
    }

    if (null != arm2d) {
      arm2d.setAngle(inputs.positionDegrees + armAngle2dOffset);
    }
  }

  private boolean isLimitHigh() {
    // if abs encoder is broken/missing then we cannot detect limits, assume we are at limit.
    if (isAbsoluteEncoderConnected() == false) {
      return true;
    }
    if (inputs.positionDegrees >= positionDegreeMax) {
      inputs.limitHigh = true;
    } else {
      inputs.limitHigh = false;
    }
    return inputs.limitHigh;
  }

  private boolean isLimitLow() {
    // if abs encoder is broken/missing then we cannot detect limits, assume we are at limit.
    if (isAbsoluteEncoderConnected() == false) {
      return true;
    }
    if (inputs.positionDegrees <= positionDegreeMin) {
      inputs.limitLow = true;

    } else {
      inputs.limitLow = false;
    }
    return inputs.limitLow;
  }

  private boolean isLimitReached(double desiredVoltage) {
    if (isLimitHigh() && desiredVoltage > 0.0) {
      return true;
    }
    if (isLimitLow() && desiredVoltage < 0.0) {
      return true;
    }
    return false;
  }

  @Override
  public boolean isAtMaxLimit() {
    return isLimitHigh();
  }

  @Override
  public boolean isAtMinLimit() {
    return isLimitLow();
  }

  @Override
  public Command getStowCommand() {
    return runOnce(() -> stow());
  }

  @Override
  public void add2dSim(Mechanism2d mech2d) {
    MechanismRoot2d armPivot2d = mech2d.getRoot("Arm Pivot", 15, 10);
    armPivot2d.append(new MechanismLigament2d("Arm Tower", 10, -90));
    arm2d =
        armPivot2d.append(
            new MechanismLigament2d(
                "Arm",
                30,
                inputs.positionDegrees + armAngle2dOffset,
                6,
                new Color8Bit(Color.kYellow)));
  }
}
