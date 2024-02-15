package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController.ArbFFUnits;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class ArmIOSparkMax implements ArmIO {
  // Leader
  private final CANSparkMax left = new CANSparkMax(4, MotorType.kBrushless);
  RelativeEncoder leftEncoder = left.getEncoder();

  // follower
  // private final CANSparkMax right = new CANSparkMax(3, MotorType.kBrushless);

  private final DutyCycleEncoder absoluteEncoder;

  private SparkPIDController leftPid = left.getPIDController();

  public double lkP, lkI, lkD, lkIz, lkFF, lkMaxOutput, lkMinOutput, lmaxRPS;

  public ArmIOSparkMax() {
    /* TODO: Instantiate 2x SparkMax motors and absolute encoder */
    absoluteEncoder = new DutyCycleEncoder(0);

    // This will need to be set from a constant once we have the arm assembled and can measure the
    // offset.  Once the arm is done this value won't change.
    //
    absoluteEncoder.setPositionOffset(0.4154156603853915); // This is place holder

    absoluteEncoder.setDutyCycleRange(1.0 / 1024.0, 1023.0 / 1024.0);

    // I don't think 2PI is correct.. try 360?
    absoluteEncoder.setDistancePerRotation(2.0 * Math.PI);
    // encoder.setDistancePerRotation(360.0);

    left.setInverted(false);
    // right.follow(left, false);

    // left.enableVoltageCompensation(12.0);
    left.setSmartCurrentLimit(40);

    left.burnFlash();

    leftEncoder.setPosition(0);

    // TODO: these values are samples picked from REV example PID code.  Need to tune PID and choose
    // real values.

    lkP = 0.1;
    lkI = 0;
    lkD = 0;
    lkIz = 0;
    lkFF = 0;
    lkMaxOutput = .5;
    lkMinOutput = -.5;
    lmaxRPS = 300;

    leftPid.setP(lkP);
    leftPid.setI(lkI);
    leftPid.setD(lkD);
    leftPid.setIZone(lkIz);
    // leftPid.setFF(lkFF);
    leftPid.setOutputRange(lkMinOutput, lkMaxOutput);

    SmartDashboard.putNumber("Arm/left/P Gain", lkP);
    SmartDashboard.putNumber("Arm/left/I Gain", lkI);
    SmartDashboard.putNumber("Arm/left/D Gain", lkD);
    SmartDashboard.putNumber("Arm/left/I Zone", lkIz);
    SmartDashboard.putNumber("Arm/left/Feed Forward", lkFF);
    SmartDashboard.putNumber("Arm/left/Max Output", lkMaxOutput);
    SmartDashboard.putNumber("Arm/left/Min Output", lkMinOutput);
  }

  /** Updates the set of loggable inputs. */
  @Override
  public void updateInputs(ArmIOInputs inputs) {

    inputs.positionRad = absoluteEncoder.getDistance();

    inputs.positionDegree = absoluteEncoder.get() * 360;

    inputs.absoluteEncoderConnected = isAbsoluteEncoderConnected();

    inputs.leftAppliedVolts = left.getAppliedOutput() * left.getBusVoltage();

    inputs.current = left.getOutputCurrent();

    inputs.relativePositionRotations = leftEncoder.getPosition();

    // Code below allows PID to be tuned using SmartDashboard.  And outputs extra data to
    // SmartDashboard.
    if (Constants.debugMode) {
      double lp = SmartDashboard.getNumber("Arm/left/P Gain", 0);
      double li = SmartDashboard.getNumber("Arm/left/I Gain", 0);
      double ld = SmartDashboard.getNumber("Arm/left/D Gain", 0);
      double liz = SmartDashboard.getNumber("Arm/left/I Zone", 0);
      double lff = SmartDashboard.getNumber("Arm/left/Feed Forward", 0);
      double lmax = SmartDashboard.getNumber("Arm/left/Max Output", 0);
      double lmin = SmartDashboard.getNumber("Arm/left/Min Output", 0);

      if ((lp != lkP)) {
        leftPid.setP(lp);
        lkP = lp;
      }
      if ((li != lkI)) {
        leftPid.setI(li);
        lkI = li;
      }
      if ((ld != lkD)) {
        leftPid.setD(ld);
        lkD = ld;
      }
      if ((liz != lkIz)) {
        leftPid.setIZone(liz);
        lkIz = liz;
      }
      if ((lff != lkFF)) {
        leftPid.setFF(lff);
        lkFF = lff;
      }
      if ((lmax != lkMaxOutput) || (lmin != lkMinOutput)) {
        leftPid.setOutputRange(lmin, lmax);
        lkMinOutput = lmin;
        lkMaxOutput = lmax;
      }

      SmartDashboard.putBoolean("Arm/absEncoder/connected", absoluteEncoder.isConnected());

      // Try rebooting the robot with the arm/encoder in different positions.  Do these value
      // change?
      // How? Record observations so you can share with rest of us
      SmartDashboard.putNumber("Arm/absEncoder/absolutePos", absoluteEncoder.getAbsolutePosition());
      SmartDashboard.putNumber(
          "Arm/absEncoder/getPositionOffset", absoluteEncoder.getPositionOffset());

      // I don't think this number changes if you rotate the arm/encoder.  What does affect it?
      SmartDashboard.putNumber(
          "Arm/absEncoder/getDistancePerRotation", absoluteEncoder.getDistancePerRotation());

      SmartDashboard.putNumber("Arm/absEncoder/get", absoluteEncoder.get());
      SmartDashboard.putNumber("Arm/absEncoder/getDistance", absoluteEncoder.getDistance());

      // Try out different encoder.get methods here and see if you can get a range of values that
      // works by applying different math/operations to the get values.  This is one example.
      SmartDashboard.putNumber("Arm/absEncoder/angle", absoluteEncoder.get() * 360.0);

      // This should show what the relative encoder is reading.  When you use the sparkmax position
      // PID it expects a setpoint in rotations.  Not clear if that means degrees or what unit is
      // used.
      SmartDashboard.putNumber("Arm/relEncoder/getPosition", leftEncoder.getPosition());
    }
  }

  @Override
  public void setPosition(double degrees, double ffVolts) {
    if (Constants.debugMode) {
      SmartDashboard.putNumber("Arm/setPosition/degrees", degrees);
      SmartDashboard.putNumber("Arm/setPosition/ffVolts", ffVolts);
    }
    leftPid.setReference(
        degrees, CANSparkMax.ControlType.kPosition, 0, ffVolts, ArbFFUnits.kVoltage);
  }

  /** Run the arm motor at the specified voltage. */
  @Override
  public void setVoltage(double volts) {
    left.setVoltage(volts);
  }

  @Override
  public void setFeedback(double kP, double kI, double kD) {
    leftPid.setP(kP);
    leftPid.setI(kI);
    leftPid.setD(kD);
  }

  @Override
  public boolean isAbsoluteEncoderConnected() {
    return absoluteEncoder.isConnected();
  }

  @Override
  public void resetRelativeEncoder(double position) {
    leftEncoder.setPosition(position);
  }

  @Override
  public void setBrakeMode(boolean brake) {
    IdleMode mode;
    if (brake) {
      mode = CANSparkMax.IdleMode.kBrake;
      SmartDashboard.putString("Arm/Idle Mode", "kBrake");
    } else {
      mode = CANSparkMax.IdleMode.kCoast;
      SmartDashboard.putString("Arm/Idle Mode", "kCoast");
    }
    if (left.setIdleMode(mode) != REVLibError.kOk) {
      SmartDashboard.putString("Arm/Idle Mode", "Error");
    }
  }
}
