package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ArmIOSparkMax implements ArmIO {
  //Leader
  private final CANSparkMax left = new CANSparkMax(2, MotorType.kBrushless);

  //follower
  private final CANSparkMax right = new CANSparkMax(3, MotorType.kBrushless);
  
  private final DutyCycleEncoder encoder;

  private SparkPIDController leftPid = left.getPIDController();

  public double lkP, lkI, lkD, lkIz, lkFF, lkMaxOutput, lkMinOutput, lmaxRPS;

  public ArmIOSparkMax() {
    /* TODO: Instantiate 2x SparkMax motors and absolute encoder */
    encoder = new DutyCycleEncoder(0);
    encoder.setPositionOffset(0); // This is place holder
    encoder.setDutyCycleRange(1 / 1025, 1024 / 1025);
    encoder.setDistancePerRotation(2 * Math.PI);

    left.setInverted(false);
    right.follow(left, false);

    left.enableVoltageCompensation(12.0);
    left.setSmartCurrentLimit(30);

    left.burnFlash();

    // TODO: these values are samples picked from REV example PID code.  Need to tune PID and choose
    // real values.

    lkP = 6e-5;
    lkI = 0;
    lkD = 0;
    lkIz = 0;
    lkFF = 0.000015;
    lkMaxOutput = 1;
    lkMinOutput = -1;
    lmaxRPS = 300;

    leftPid.setP(lkP);
    leftPid.setI(lkI);
    leftPid.setD(lkD);
    leftPid.setIZone(lkIz);
    leftPid.setFF(lkFF);
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
    /* TODO: Implement SparkMax/Absolute Encoder Code Here */
    inputs.positionRad = Units.rotationsToRadians(encoder.getAbsolutePosition());

    inputs.leftAppliedVolts = left.getAppliedOutput() * left.getBusVoltage();
    inputs.rightAppliedVolts = right.getAppliedOutput() * right.getBusVoltage();

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
  }

  @Override
  public void setPosition(double radians, double ffVolts) {

    leftPid.setReference(
        Units.radiansToRotations(radians),
        CANSparkMax.ControlType.kPosition,
        0, // Arbitrary slotID, you may need to adjust this based on your configuration
        ffVolts);

    SmartDashboard.putNumber("Shooter/left/positionRad", radians);
    SmartDashboard.putNumber("Shooter/left/ProcessVariable", encoder.getAbsolutePosition());
  }

  /** Run the arm motor at the specified voltage. */
  @Override
  public void setVoltage(double volts) {
    /* TODO: Implement SparkMax Code Here */
    left.setVoltage(volts);
  }
}
