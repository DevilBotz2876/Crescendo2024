package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.util.LoggedTunableNumber;
import java.util.Map;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class ArmSubsystem extends SubsystemBase implements Arm {
  private final ArmIO io;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
  private ArmFeedforward feedforward;
  private final SysIdRoutine sysId;
  private final double positionRadMax = 1.3;
  private final double positionRadMin = 0.001;

  @AutoLogOutput private double desiredVoltage;
  @AutoLogOutput private double setPoint;

  // Create a Mechanism2d display of an Arm with a fixed ArmTower and moving Arm.
  private final Mechanism2d mech2d = new Mechanism2d(60, 60);
  private final MechanismRoot2d armPivot2d = mech2d.getRoot("ArmPivot", 30, 30);
  private final MechanismLigament2d armTower2d =
      armPivot2d.append(new MechanismLigament2d("ArmTower", 30, -90));
  private final MechanismLigament2d arm2d =
      armPivot2d.append(
          new MechanismLigament2d(
              "Arm",
              30,
              Units.radiansToDegrees(inputs.positionRad),
              6,
              new Color8Bit(Color.kYellow)));

  private static final LoggedTunableNumber armKp = new LoggedTunableNumber("Arm/kP");
  private static final LoggedTunableNumber armKd = new LoggedTunableNumber("Arm/kD");
  private static final LoggedTunableNumber armKg = new LoggedTunableNumber("Arm/kG");
  private static final LoggedTunableNumber armKv = new LoggedTunableNumber("Arm/kV");
  private static final LoggedTunableNumber armKa = new LoggedTunableNumber("Arm/kA");

  ShuffleboardTab armTab;
  GenericEntry armVoltsEntry;
  GenericEntry armDegreesEntry;
  GenericEntry highLimitEntry;
  GenericEntry lowLimitEntry;

  private double kG, kV, kA;

  public ArmSubsystem(ArmIO io) {
    this.io = io;

    armKp.initDefault(.1);
    armKd.initDefault(0.0);
    armKg.initDefault(.72);
    armKv.initDefault(6.18);
    armKa.initDefault(.04);

    kG = armKg.get();
    kV = armKv.get();
    kA = armKa.get();

    // create arm tab on ShuffleBoard
    armTab = Shuffleboard.getTab("Arm");
    // Create volt entry under arm tab as a number sider with min = -4 and max = 4
    armVoltsEntry =
        armTab
            .add("Volts", 0)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", -4, "max", 4))
            .getEntry();

    highLimitEntry =
        armTab.add("HighLimit", false).withWidget(BuiltInWidgets.kBooleanBox).getEntry();

    lowLimitEntry = armTab.add("LowLimit", false).withWidget(BuiltInWidgets.kBooleanBox).getEntry();

    // Configure SysId based on the AdvantageKit example
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Arm/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism((voltage) -> runVoltage(voltage.in(Volts)), null, this));

    SmartDashboard.putData("Arm Simulation", mech2d);
  }

  @Override
  public double getAngle() {
    /* TODO */
    return inputs.positionDegree;
    // return 0;
  }

  // sets of the angle of the arm
  @Override
  public void setAngle(double degrees) {
    /* TODO: Enforce arm physical min/max limits */
    final double minAngleDeg = 0;
    final double maxAngleDeg = 103;

    // Check if the angle is below the minimum limit or above the maximum limit
    // If it is the it is set to min/max
    if (degrees < minAngleDeg) {
      this.setPoint = minAngleDeg; // Set to the minimum angle
    } else if (degrees > maxAngleDeg) {
      this.setPoint = maxAngleDeg; // Set to the maximum angle
    } else {
      // The  angle is within the range and is set
      this.setPoint = degrees;
    }

    feedforward = new ArmFeedforward(0, kG, kV, kA);

    // Set the position reference with feedforward voltage
    io.setPosition(this.setPoint, feedforward.calculate(this.setPoint, 0));
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
    if (armKp.hasChanged(hashCode()) || armKd.hasChanged(hashCode())) {
      io.setFeedback(armKp.get(), 0.0, armKd.get());
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

    if (isHighLimit()) {
      // TODO: turn off voltage or stop pid
    }
    if (isLowLimit()) {
      // TODO: turn off voltage or stop pid
      io.resetRelativeEncoder(0.0);
    }

    arm2d.setAngle(inputs.positionDegree);
  }

  private boolean isHighLimit() {
    if (inputs.positionRad > positionRadMax) {
      highLimitEntry.setBoolean(true);
      inputs.highLimit = true;
    } else {
      highLimitEntry.setBoolean(false);
      inputs.highLimit = false;
    }
    return inputs.highLimit;
  }

  private boolean isLowLimit() {
    if (inputs.positionRad < positionRadMin) {
      inputs.lowLimit = true;
      lowLimitEntry.setBoolean(true);
    } else {
      lowLimitEntry.setBoolean(false);
      inputs.lowLimit = false;
    }
    return inputs.lowLimit;
  }

  private boolean isLimitReached(double desiredVoltage) {
    if (isHighLimit() && desiredVoltage > 0.0) {
      return true;
    }
    if (isLowLimit() && desiredVoltage < 0.0) {
      return true;
    }
    return false;
  }
}
