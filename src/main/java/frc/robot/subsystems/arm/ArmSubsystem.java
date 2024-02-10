package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class ArmSubsystem extends SubsystemBase implements Arm {
  private final ArmIO io;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
  @AutoLogOutput private double degrees = 0;
  ArmFeedforward feedforward;
  private final SysIdRoutine sysId;
  private final double positionRadMax = 3.160;
  private final double positionRadMin = 0.001;
  @AutoLogOutput private double volts;

  public ArmSubsystem(ArmIO io) {
    this.io = io;

    // TODO: These are sample values.  Need to run sysid on shooter and get real values.
    // feedforward = new ArmFeedforward(1, 1, 1);

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
  public double getAngle() {
    /* TODO */
    return Units.radiansToDegrees(inputs.positionRad);
    // return 0;
  }

  // sets of the angle of the arm
  @Override
  public void setAngle(double degrees) {
    /* TODO: Enforce arm physical min/max limits */
    final double minAngleDeg = 0;
    final double maxAngleDeg = 110;

    // Check if the angle is below the minimum limit or above the maximum limit
    // If it is the it is set to min/max
    if (degrees < minAngleDeg) {
      this.degrees = minAngleDeg; // Set to the minimum angle
    } else if (degrees > maxAngleDeg) {
      this.degrees = maxAngleDeg; // Set to the maximum angle
    }
    // The  angle is within the range and is set
    else {
      this.degrees = degrees;
    }

    // target position for
    double setpoint = degrees;

    ArmFeedforward feedforward = new ArmFeedforward(0, 0.72, 6.18, 0.04);

    // Set the position reference with feedforward voltage
    io.setPosition(setpoint, feedforward.calculate(setpoint, 0));
  }

  // Sets the voltage to volts. the volts value is -12 to 12
  public void runVoltage(double volts) {
    this.volts = volts;
    io.setVoltage(voltageSafety(volts));
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
    // Updates the inputs
    io.updateInputs(inputs);
    Logger.processInputs("Arm", inputs);
    SmartDashboard.putNumber("Arm/encoder/anglSub", Units.radiansToDegrees(inputs.positionRad));

    /* TODO: Implement PID control here to achieve desired angle */
    // if (isLimitReached(inputs.leftAppliedVolts)) {
    //   runVoltage(voltageSafety(inputs.leftAppliedVolts));
    // }
  }

  protected boolean isLimitReached(double desiredVoltage) {
    boolean limitHit = false;
    if (inputs.positionRad > positionRadMax && desiredVoltage > 0.0) {
      //System.out.println("Max Hit");
      SmartDashboard.putBoolean("Arm/maxHit", true);
      limitHit = true;
    } else {
      SmartDashboard.putBoolean("Arm/maxHit", false);
    }
    if (inputs.positionRad < positionRadMin && desiredVoltage < 0.0) {
      //System.out.println("Min Hit");
      SmartDashboard.putBoolean("Arm/minHit", true);
      limitHit = true;
      io.resetPosition();
    } else {
      SmartDashboard.putBoolean("Arm/minHit", false);
    }
    return limitHit;
  }

  protected double voltageSafety(double desiredVoltage) {
    if (isLimitReached(desiredVoltage)) return 0.0;
    else return desiredVoltage;
  }
}
