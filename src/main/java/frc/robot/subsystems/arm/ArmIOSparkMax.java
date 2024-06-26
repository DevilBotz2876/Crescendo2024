package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController.ArbFFUnits;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.config.RobotConfig;
import frc.robot.config.RobotConfig.ArmConstants;

public class ArmIOSparkMax implements ArmIO {

  private final CANSparkMax motor;
  private final RelativeEncoder relEncoder;
  private final DutyCycleEncoder absoluteEncoder;
  private final SparkPIDController armPid;
  public double lkP, lkI, lkD, lkIz, lkFF, lkMaxOutput, lkMinOutput, lmaxRPS;

  public ArmIOSparkMax(int id) {
    this(id, false);
  }

  double getOffsetCorrectedAbsolutePositionInRadians() {
    return ((absoluteEncoder.getAbsolutePosition() - ArmConstants.absolutePositionOffset)
            * ArmConstants.absoluteEncoderInversion)
        * 2.0
        * Math.PI;
  }

  public ArmIOSparkMax(int id, boolean inverted) {
    /* Instantiate 1x SparkMax motors and absolute encoder */
    motor = new CANSparkMax(id, MotorType.kBrushless);

    // first thing we do to spark device is reset it to known defaults.
    motor.restoreFactoryDefaults();

    relEncoder = motor.getEncoder();
    armPid = motor.getPIDController();
    absoluteEncoder = new DutyCycleEncoder(0);

    // This will need to be set from a constant once we have the arm assembled and can measure the
    // offset.  Once the arm is done this value won't change.  It can change if arm chain slips so
    // check it after any mechanican work is done.  Also the decimal places matter.  Don't round or
    // leave off numbers.
    //
    System.out.println("ArmIOSparkMax(): Absolute Position Offset: " + absoluteEncoder.get());
    absoluteEncoder.setDutyCycleRange(1.0 / 1024.0, 1023.0 / 1024.0);

    motor.setInverted(inverted);

    // motor.enableVoltageCompensation(12.0);
    motor.setSmartCurrentLimit(40);

    // Initialize the relative encoder position based on absolute encoder position.  The abs and rel
    // encoder do not scale/align 1-1. At zero they are both zero.  When rel encoder is 80, abs
    // encoder is not. Perhaps we just say that if abs encoder is at 80, set rel to X, or abs
    // encoder is at 0, set rel to 0.  Everything else is invalid and requires arm to rehome itself.
    //
    // relEncoder.setPosition(0);

    // 60:1 gear box, 72 teeth on the arm cog and 14 teeth on the motor cog
    double gearRatio = (60.0 * (72.0 / 14.0));
    double rotationsToDegreesConversionFactor = 360.0 / gearRatio;
    relEncoder.setPositionConversionFactor(rotationsToDegreesConversionFactor);
    relEncoder.setVelocityConversionFactor(rotationsToDegreesConversionFactor / 60.0);
    relEncoder.setPosition(Units.radiansToDegrees(getOffsetCorrectedAbsolutePositionInRadians()));

    lkP = RobotConfig.ArmConstants.pidKp;
    lkI = RobotConfig.ArmConstants.pidKi;
    lkD = RobotConfig.ArmConstants.pidKd;
    lkIz = 0;
    lkFF = 0;
    lkMaxOutput = RobotConfig.ArmConstants.pidMaxOutput;
    lkMinOutput = RobotConfig.ArmConstants.pidMinOutput;
    lmaxRPS = 300;

    armPid.setP(lkP);
    armPid.setI(lkI);
    armPid.setD(lkD);
    armPid.setIZone(lkIz);
    armPid.setFF(lkFF);
    armPid.setOutputRange(lkMinOutput, lkMaxOutput);

    // SmartDashboard.putNumber("Arm/pid/P Gain", lkP);
    // SmartDashboard.putNumber("Arm/pid/I Gain", lkI);
    // SmartDashboard.putNumber("Arm/pid/D Gain", lkD);
    // SmartDashboard.putNumber("Arm/pid/I Zone", lkIz);
    // SmartDashboard.putNumber("Arm/pid/Feed Forward", lkFF);
    // SmartDashboard.putNumber("Arm/pid/Max Output", lkMaxOutput);
    // SmartDashboard.putNumber("Arm/pid/Min Output", lkMinOutput);

    // Last thing we do is burn config to spark flash
    motor.burnFlash();
  }

  /** Updates the set of loggable inputs. */
  @Override
  public void updateInputs(ArmIOInputs inputs) {
    inputs.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
    inputs.currentAmps = motor.getOutputCurrent();
    inputs.positionDegrees = Units.radiansToDegrees(getOffsetCorrectedAbsolutePositionInRadians());
    inputs.velocityDegrees = relEncoder.getVelocity();

    inputs.absoluteEncoderConnected = isAbsoluteEncoderConnected();
    inputs.absolutePositionRaw = absoluteEncoder.getAbsolutePosition();
    inputs.relativePositionDegrees = relEncoder.getPosition();
    inputs.positionError = inputs.positionDegrees - inputs.relativePositionDegrees;

    // Code below allows PID to be tuned using SmartDashboard.  And outputs extra data to
    // SmartDashboard.
    if (Constants.debugMode) {
      //   double lp = SmartDashboard.getNumber("Arm/pid/P Gain", 0);
      //   double li = SmartDashboard.getNumber("Arm/pid/I Gain", 0);
      //   double ld = SmartDashboard.getNumber("Arm/pid/D Gain", 0);
      //   double liz = SmartDashboard.getNumber("Arm/pid/I Zone", 0);
      //   double lff = SmartDashboard.getNumber("Arm/pid/Feed Forward", 0);
      //   double lmax = SmartDashboard.getNumber("Arm/pid/Max Output", 0);
      //   double lmin = SmartDashboard.getNumber("Arm/pid/Min Output", 0);

      //   if ((lp != lkP)) {
      //     armPid.setP(lp);
      //     lkP = lp;
      //   }
      //   if ((li != lkI)) {
      //     armPid.setI(li);
      //     lkI = li;
      //   }
      //   if ((ld != lkD)) {
      //     armPid.setD(ld);
      //     lkD = ld;
      //   }
      //   if ((liz != lkIz)) {
      //     armPid.setIZone(liz);
      //     lkIz = liz;
      //   }
      //   if ((lff != lkFF)) {
      //     armPid.setFF(lff);
      //     lkFF = lff;
      //   }
      //   if ((lmax != lkMaxOutput) || (lmin != lkMinOutput)) {
      //     armPid.setOutputRange(lmin, lmax);
      //     lkMinOutput = lmin;
      //     lkMaxOutput = lmax;
      //   }

      SmartDashboard.putBoolean("Arm/absEncoder/connected", absoluteEncoder.isConnected());

      // Try rebooting the robot with the arm/encoder in different positions.  Do these value
      // change?
      // How? Record observations so you can share with rest of us
      SmartDashboard.putNumber("Arm/absEncoder/absolutePos", absoluteEncoder.getAbsolutePosition());

      // This should show what the relative encoder is reading.  When you use the sparkmax position
      // PID it expects a setpoint in rotations.  Not clear if that means degrees or what unit is
      // used.
      SmartDashboard.putNumber("Arm/relEncoder/getPosition", relEncoder.getPosition());
    }
  }

  @Override
  public void setPosition(double degrees, double ffVolts) {
    if (Constants.debugMode) {
      SmartDashboard.putNumber("Arm/setPosition/degrees", degrees);
      SmartDashboard.putNumber("Arm/setPosition/ffVolts", ffVolts);
    }
    armPid.setReference(
        degrees, CANSparkMax.ControlType.kPosition, 0, ffVolts, ArbFFUnits.kVoltage);
  }

  /** Run the arm motor at the specified voltage. */
  @Override
  public void setVoltage(double volts) {
    motor.setVoltage(volts);
  }

  @Override
  public void setFeedback(double kP, double kI, double kD, double minOutput, double maxOutput) {
    armPid.setP(kP);
    armPid.setI(kI);
    armPid.setD(kD);
    armPid.setOutputRange(minOutput, maxOutput);
  }

  @Override
  public boolean isAbsoluteEncoderConnected() {
    return absoluteEncoder.isConnected();
  }

  @Override
  public void resetRelativeEncoder(double position) {
    // System.out.println("resetRelativeEncoder " + position);
    relEncoder.setPosition(position);
  }

  @Override
  public void setBrakeMode(boolean brake) {
    IdleMode mode;
    if (brake) {
      mode = CANSparkMax.IdleMode.kBrake;
      if (Constants.debugMode) {
        SmartDashboard.putString("Arm/Idle Mode", "kBrake");
      }
    } else {
      mode = CANSparkMax.IdleMode.kCoast;
      if (Constants.debugMode) {
        SmartDashboard.putString("Arm/Idle Mode", "kCoast");
      }
    }
    if (motor.setIdleMode(mode) != REVLibError.kOk) {
      if (Constants.debugMode) {
        SmartDashboard.putString("Arm/Idle Mode", "Error");
      }
    }
  }
}
