package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ArmIOSparkMax implements ArmIO {
  private final CANSparkMax left = new CANSparkMax(2, MotorType.kBrushless);

  private final CANSparkMax right = new CANSparkMax(3, MotorType.kBrushless);
  ;
  private final DutyCycleEncoder encoder;

  private SparkPIDController leftPid = left.getPIDController();
  private SparkPIDController rightPid = right.getPIDController();

  public double lkP, lkI, lkD, lkIz, lkFF, lkMaxOutput, lkMinOutput, lmaxRPS;
  public double rkP, rkI, rkD, rkIz, rkFF, rkMaxOutput, rkMinOutput, rmaxRPS;

  public ArmIOSparkMax() {
    /* TODO: Instantiate 2x SparkMax motors and absolute encoder */
    encoder = new DutyCycleEncoder(0);
    encoder.setPositionOffset(0); // This is place holder
    encoder.setDutyCycleRange(1, 1024);
    encoder.setDistancePerRotation(360);

    right.setInverted(false);

    left.setInverted(false);

    left.enableVoltageCompensation(12.0);
    left.setSmartCurrentLimit(30);

    left.burnFlash();

    // TODO: these values are samples picked from REV example PID code.  Need to tune PID and choose
    // real values.

    // left
    lkP = 6e-5;
    lkI = 0;
    lkD = 0;
    lkIz = 0;
    lkFF = 0.000015;
    lkMaxOutput = 1;
    lkMinOutput = -1;
    rmaxRPS = 300;

    // right
    rkP = 6e-5;
    rkI = 0;
    rkD = 0;
    rkIz = 0;
    rkFF = 0.000015;
    rkMaxOutput = 1;
    rkMinOutput = 0;
    rmaxRPS = 300;

    // left
    leftPid.setP(lkP);
    leftPid.setI(lkI);
    leftPid.setD(lkD);
    leftPid.setIZone(lkIz);
    leftPid.setFF(lkFF);
    leftPid.setOutputRange(lkMinOutput, lkMaxOutput);

    // TODO: probably remove right since Arm will have one motor, not two independent motors
    rightPid.setP(rkP);
    rightPid.setI(rkI);
    rightPid.setD(rkD);
    rightPid.setIZone(rkIz);
    rightPid.setFF(rkFF);
    rightPid.setOutputRange(rkMinOutput, rkMaxOutput);

    // left
    SmartDashboard.putNumber("Arm/left/P Gain", lkP);
    SmartDashboard.putNumber("Arm/left/I Gain", lkI);
    SmartDashboard.putNumber("Arm/left/D Gain", lkD);
    SmartDashboard.putNumber("Arm/left/I Zone", lkIz);
    SmartDashboard.putNumber("Arm/left/Feed Forward", lkFF);
    SmartDashboard.putNumber("Arm/left/Max Output", lkMaxOutput);
    SmartDashboard.putNumber("Arm/left/Min Output", lkMinOutput);

    // TODO: probably remove this since Arm will have one motor, not two independent motors
    //
    SmartDashboard.putNumber("Arm/right/P Gain", rkP);
    SmartDashboard.putNumber("Arm/right/I Gain", rkI);
    SmartDashboard.putNumber("Arm/right/D Gain", rkD);
    SmartDashboard.putNumber("Arm/right/I Zone", rkIz);
    SmartDashboard.putNumber("Arm/right/Feed Forward", rkFF);
    SmartDashboard.putNumber("Arm/right/Max Output", rkMaxOutput);
    SmartDashboard.putNumber("Arm/right/Min Output", rkMinOutput);
  }

  /** Updates the set of loggable inputs. */
  @Override
  public void updateInputs(ArmIOInputs inputs) {
    /* TODO: Implement SparkMax/Absolute Encoder Code Here */
    inputs.positionRad = Units.rotationsToRadians(encoder.getAbsolutePosition());

    inputs.leftAppliedVolts = left.getAppliedOutput() * left.getBusVoltage();
    inputs.rightAppliedVolts = right.getAppliedOutput() * right.getBusVoltage();

    // left
    double lp = SmartDashboard.getNumber("Arm/left/P Gain", 0);
    double li = SmartDashboard.getNumber("Arm/left/I Gain", 0);
    double ld = SmartDashboard.getNumber("Arm/left/D Gain", 0);
    double liz = SmartDashboard.getNumber("Arm/left/I Zone", 0);
    double lff = SmartDashboard.getNumber("Arm/left/Feed Forward", 0);
    double lmax = SmartDashboard.getNumber("Arm/left/Max Output", 0);
    double lmin = SmartDashboard.getNumber("Arm/left/Min Output", 0);

    double rp = SmartDashboard.getNumber("Arm/right/P Gain", 0);
    double ri = SmartDashboard.getNumber("Arm/right/I Gain", 0);
    double rd = SmartDashboard.getNumber("Arm/right/D Gain", 0);
    double riz = SmartDashboard.getNumber("Arm/right/I Zone", 0);
    double rff = SmartDashboard.getNumber("Arm/right/Feed Forward", 0);
    double rmax = SmartDashboard.getNumber("Arm/right/Max Output", 0);
    double rmin = SmartDashboard.getNumber("Arm/right/Min Output", 0);

    // left
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

    // right
    if ((rp != rkP)) {
      rightPid.setP(rp);
      rkP = rp;
    }
    if ((ri != rkI)) {
      rightPid.setI(ri);
      rkI = ri;
    }
    if ((rd != rkD)) {
      rightPid.setD(rd);
      rkD = rd;
    }
    if ((riz != rkIz)) {
      rightPid.setIZone(riz);
      rkIz = riz;
    }
    if ((rff != rkFF)) {
      rightPid.setFF(rff);
      rkFF = rff;
    }
    if ((rmax != rkMaxOutput) || (rmin != rkMinOutput)) {
      rightPid.setOutputRange(rmin, rmax);
      rkMinOutput = rmin;
      rkMaxOutput = rmax;
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

    rightPid.setReference(
        Units.radiansToRotations(radians),
        CANSparkMax.ControlType.kPosition,
        0, // Arbitrary slotID, you may need to adjust this based on your configuration
        ffVolts);
    SmartDashboard.putNumber("Shooter/right/positionRad", radians);
    SmartDashboard.putNumber("Shooter/right/ProcessVariable", encoder.getAbsolutePosition());
  }

  /** Run the arm motor at the specified voltage. */
  @Override
  public void setVoltage(double volts) {
    /* TODO: Implement SparkMax Code Here */
    left.setVoltage(volts);
    right.setVoltage(volts);
  }
}
