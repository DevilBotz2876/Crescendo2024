package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.config.RobotConfig.ArmConstants;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class ArmSubsystem extends SubsystemBase implements Arm {
  private final ArmIO io;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
  private ArmFeedforward feedforward;
  private final SysIdRoutine sysId;
  private final double positionDegreeMax = ArmConstants.maxAngleInDegrees;
  private final double positionDegreeMin = ArmConstants.minAngleInDegrees;
  private TrapezoidProfile.State currentState = new TrapezoidProfile.State();
  @AutoLogOutput private double desiredVoltage;
  @AutoLogOutput private double setPoint;

  // Create a Mechanism2d display of an Arm with a fixed ArmTower and moving Arm.
  private final double armAngle2dOffset = -45;
  private final MechanismLigament2d arm2d;

  private static final LoggedTunableNumber armKp = new LoggedTunableNumber("Arm/pid/kP");
  private static final LoggedTunableNumber armKd = new LoggedTunableNumber("Arm/pid/kD");

  private static final LoggedTunableNumber armKg = new LoggedTunableNumber("Arm/pid/kG");
  private static final LoggedTunableNumber armKv = new LoggedTunableNumber("Arm/pid/kV");
  private static final LoggedTunableNumber armKa = new LoggedTunableNumber("Arm/pid/kA");

  private static final LoggedTunableNumber armOutputMax =
      new LoggedTunableNumber("Arm/pid/outputMax");
  private static final LoggedTunableNumber armOutputMin =
      new LoggedTunableNumber("Arm/pid/outputMin");

  private static final LoggedTunableNumber armMaxVelocity =
      new LoggedTunableNumber("Arm/constraints/maxVelocity");
  private static final LoggedTunableNumber armMaxAccel =
      new LoggedTunableNumber("Arm/constraints/minAccel");

  private double kG, kV, kA;

  private boolean relEncoderInit;

  public ArmSubsystem(ArmIO io) {
    this.io = io;

    armKp.initDefault(ArmConstants.pidKp);
    armKd.initDefault(ArmConstants.pidKd);
    armKg.initDefault(ArmConstants.ffKg);
    armKv.initDefault(ArmConstants.ffKv);
    armKa.initDefault(ArmConstants.ffKa);
    armOutputMax.initDefault(ArmConstants.pidMaxOutput);
    armOutputMin.initDefault(ArmConstants.pidMinOutput);

    armMaxVelocity.initDefault(ArmConstants.maxVelocity);
    armMaxAccel.initDefault(ArmConstants.maxAcceleration);

    kG = armKg.get();
    kV = armKv.get();
    kA = armKa.get();

    // Configure SysId based on the AdvantageKit example
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Arm/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism((voltage) -> runVoltage(voltage.in(Volts)), null, this));

    Mechanism2d mech2d = new Mechanism2d(60, 60);
    MechanismRoot2d armPivot2d = mech2d.getRoot("Arm Pivot", 10, 30);
    armPivot2d.append(new MechanismLigament2d("Arm Tower", 20, -90));
    arm2d =
        armPivot2d.append(
            new MechanismLigament2d(
                "Arm",
                40,
                inputs.absolutePositionDegree + armAngle2dOffset,
                6,
                new Color8Bit(Color.kYellow)));

    SmartDashboard.putData("Arm Simulation", mech2d);

    relEncoderInit = true;
  }

  public TrapezoidProfile.Constraints getConstraints() {
    double maxV = ArmConstants.maxVelocity;
    double maxA = ArmConstants.maxAcceleration;
    if (armMaxVelocity.hasChanged(hashCode())) {
      maxV = armMaxVelocity.get();
    }
    if (armMaxAccel.hasChanged(hashCode())) {
      maxA = armMaxAccel.get();
    }
    return new TrapezoidProfile.Constraints(maxV, maxA);
  }

  public void setState(TrapezoidProfile.State state) {
    currentState = state;

    Logger.recordOutput("Arm/state/position", state.position);
    Logger.recordOutput("Arm/state/velocity", state.velocity);

    setAngle(state.position);
  }

  public TrapezoidProfile.State getState() {
    return currentState;
  }

  @Override
  public double getAngle() {
    return inputs.absolutePositionDegree;
  }

  @Override
  public double getRelativeAngle() {
    return inputs.relativePositionDegrees;
  }

  @Override
  public double getTargetAngle() {
    return setPoint;
  }

  // sets of the angle of the arm
  @Override
  public void setAngle(double degrees) {
    Logger.recordOutput("Arm/setAngle/requestedAngleDegress", degrees);
    // Don't try to set position if absolute encoder is broken/missing.
    if (isAbsoluteEncoderConnected() == false) {
      return;
    }
    // Check if the arm angle is within limits.  Don't try to move the arm to new angle if it is
    // already at limit.
    // if (isHighLimit() || isLowLimit()) {
    //   return;
    // }

    // Check if the angle is below the minimum limit or above the maximum limit
    // If it is the it is set to min/max
    if (degrees < ArmConstants.minAngleInDegrees) {
      this.setPoint = ArmConstants.minAngleInDegrees; // Set to the minimum angle
    } else if (degrees > ArmConstants.maxAngleInDegrees) {
      this.setPoint = ArmConstants.maxAngleInDegrees; // Set to the maximum angle
    } else {
      // The  angle is within the range and is set
      this.setPoint = degrees;
    }

    // We instantiate a new object here each time because constants can change when being tuned.
    feedforward = new ArmFeedforward(0, kG, kV, kA);
    double ff = feedforward.calculate(this.setPoint, 0);

    Logger.recordOutput("Arm/setAngle/setpointDegrees", this.setPoint);
    Logger.recordOutput("Arm/setAngle/ffVolts", ff);

    // Set the position reference with feedforward voltage
    io.setPosition(this.setPoint, ff);
  }

  @Override
  public boolean isAbsoluteEncoderConnected() {
    return io.isAbsoluteEncoderConnected();
  }

  // Sets the voltage to volts. the volts value is -12 to 12
  public void runVoltage(double volts) {
    desiredVoltage = voltageSafety(volts);
    io.setVoltage(desiredVoltage);
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
    if (armKp.hasChanged(hashCode())
        || armKd.hasChanged(hashCode())
        || armOutputMin.hasChanged(hashCode())
        || armOutputMax.hasChanged(hashCode())) {
      io.setFeedback(armKp.get(), 0.0, armKd.get(), armOutputMin.get(), armOutputMax.get());
    }
    if (armKg.hasChanged(hashCode())) {
      kG = armKg.get();
    }
    if (armKv.hasChanged(hashCode())) {
      kV = armKv.get();
    }
    if (armKa.hasChanged(hashCode())) {
      kA = armKa.get();
    }
    // Updates the inputs
    io.updateInputs(inputs);
    Logger.processInputs("Arm", inputs);

    // The relative encoder is initialized in the hw specific code/file.
    //
    // if (relEncoderInit) {
    //   io.resetRelativeEncoder(
    //       0); // TODO: We need to figure out the mapping for the absolute encoder to relative
    //   // encoder
    //   relEncoderInit = false;
    // }

    if (isLimitHigh() && inputs.appliedVolts > 0) {
      // TODO: turn off voltage or stop pid
      io.setVoltage(0);
    }
    if (isLimitLow() && inputs.appliedVolts < 0) {
      // TODO: turn off voltage or stop pid
      // io.resetRelativeEncoder(0.0);
      io.setVoltage(0);
    }

    arm2d.setAngle(inputs.absolutePositionDegree + armAngle2dOffset);
  }

  private boolean isLimitHigh() {
    // if abs encoder is broken/missing then we cannot detect limits, assume we are at limit.
    if (isAbsoluteEncoderConnected() == false) {
      return true;
    }
    if (inputs.absolutePositionDegree >= positionDegreeMax) {
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
    if (inputs.absolutePositionDegree <= positionDegreeMin) {
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

  public Command getStowCommand() {
    return runOnce(() -> stow());
  }
}
