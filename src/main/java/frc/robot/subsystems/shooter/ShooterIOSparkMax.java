package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController.ArbFFUnits;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShooterIOSparkMax implements ShooterIO {
  // Gear ratio for the shooter mechanism
  private static final double GEAR_RATIO = 1.0;

  // define the 2 SparkMax Controllers. A top, and a bottom
  private final CANSparkMax top = new CANSparkMax(2, MotorType.kBrushless);
  private final CANSparkMax bottom = new CANSparkMax(1, MotorType.kBrushless);

  private SparkPIDController topPid = top.getPIDController();
  private SparkPIDController bottomPid = bottom.getPIDController();

  public double tkP, tkI, tkD, tkIz, tkFF, tkMaxOutput, tkMinOutput, tmaxRPS;

  // TODO: probably remove bottom since shooter will have one motor, not two independent motors
  public double bkP, bkI, bkD, bkIz, bkFF, bkMaxOutput, bkMinOutput, bmaxRPS;

  // Gets the NEO encoder
  private final RelativeEncoder topEncoder = top.getEncoder();
  private final RelativeEncoder bottomEncoder = top.getEncoder();

  public ShooterIOSparkMax() {

    bottom.setInverted(false);

    top.setInverted(false);

    top.enableVoltageCompensation(12.0);
    top.setSmartCurrentLimit(30);

    top.burnFlash();

    // TODO: these values are samples picked from REV example PID code.  Need to tune PID and choose
    // real values.
    tkP = 6e-5;
    tkI = 0;
    tkD = 0;
    tkIz = 0;
    tkFF = 0.000015;
    tkMaxOutput = 1;
    tkMinOutput = -1;
    tmaxRPS = 300;

    // TODO: probably remove bottom since shooter will have one motor, not two independent motors
    bkP = 6e-5;
    bkI = 0;
    bkD = 0;
    bkIz = 0;
    bkFF = 0.000015;
    bkMaxOutput = 1;
    bkMinOutput = 0;
    bmaxRPS = 300;

    topPid.setP(tkP);
    topPid.setI(tkI);
    topPid.setD(tkD);
    topPid.setIZone(tkIz);
    topPid.setFF(tkFF);
    topPid.setOutputRange(tkMinOutput, tkMaxOutput);

    // TODO: probably remove bottom since shooter will have one motor, not two independent motors
    bottomPid.setP(bkP);
    bottomPid.setI(bkI);
    bottomPid.setD(bkD);
    bottomPid.setIZone(bkIz);
    bottomPid.setFF(bkFF);
    bottomPid.setOutputRange(bkMinOutput, bkMaxOutput);

    SmartDashboard.putNumber("Shooter/top/P Gain", tkP);
    SmartDashboard.putNumber("Shooter/top/I Gain", tkI);
    SmartDashboard.putNumber("Shooter/top/D Gain", tkD);
    SmartDashboard.putNumber("Shooter/top/I Zone", tkIz);
    SmartDashboard.putNumber("Shooter/top/Feed Forward", tkFF);
    SmartDashboard.putNumber("Shooter/top/Max Output", tkMaxOutput);
    SmartDashboard.putNumber("Shooter/top/Min Output", tkMinOutput);

    // TODO: probably remove this since shooter will have one motor, not two independent motors
    //
    // SmartDashboard.putNumber("Shooter/bot/P Gain", bkP);
    // SmartDashboard.putNumber("Shooter/bot/I Gain", bkI);
    // SmartDashboard.putNumber("Shooter/bot/D Gain", bkD);
    // SmartDashboard.putNumber("Shooter/bot/I Zone", bkIz);
    // SmartDashboard.putNumber("Shooter/bot/Feed Forward", bkFF);
    // SmartDashboard.putNumber("Shooter/bot/Max Output", bkMaxOutput);
    // SmartDashboard.putNumber("Shooter/bot/Min Output", bkMinOutput);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    // Set velocityRadPerSec to the encoder velocity(rotationsPerMinute) divided by the gear ratio
    // and converted into Radians Per Second
    inputs.velocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(topEncoder.getVelocity() / GEAR_RATIO);
    // Get applied voltage from the top motor
    inputs.appliedVolts = top.getAppliedOutput() * top.getBusVoltage();

    //    inputs.velocityRadPerSecBottom =
    //        Units.rotationsPerMinuteToRadiansPerSecond(bottomEncoder.getVelocity() / GEAR_RATIO);
    // Get applied voltage from the top motor
    //    inputs.appliedVoltsBottom = bottom.getAppliedOutput() * bottom.getBusVoltage();

    double tp = SmartDashboard.getNumber("Shooter/top/P Gain", 0);
    double ti = SmartDashboard.getNumber("Shooter/top/I Gain", 0);
    double td = SmartDashboard.getNumber("Shooter/top/D Gain", 0);
    double tiz = SmartDashboard.getNumber("Shooter/top/I Zone", 0);
    double tff = SmartDashboard.getNumber("Shooter/top/Feed Forward", 0);
    double tmax = SmartDashboard.getNumber("Shooter/top/Max Output", 0);
    double tmin = SmartDashboard.getNumber("Shooter/top/Min Output", 0);

    // TODO: probably remove this since shooter will have one motor, not two independent motors
    //
    // double bp = SmartDashboard.getNumber("Shooter/bot/P Gain", 0);
    // double bi = SmartDashboard.getNumber("Shooter/bot/I Gain", 0);
    // double bd = SmartDashboard.getNumber("Shooter/bot/D Gain", 0);
    // double biz = SmartDashboard.getNumber("Shooter/bot/I Zone", 0);
    // double bff = SmartDashboard.getNumber("Shooter/bot/Feed Forward", 0);
    // double bmax = SmartDashboard.getNumber("Shooter/bot/Max Output", 0);
    // double bmin = SmartDashboard.getNumber("Shooter/bot/Min Output", 0);

    if ((tp != tkP)) {
      topPid.setP(tp);
      tkP = tp;
    }
    if ((ti != tkI)) {
      topPid.setI(ti);
      tkI = ti;
    }
    if ((td != tkD)) {
      topPid.setD(td);
      tkD = td;
    }
    if ((tiz != tkIz)) {
      topPid.setIZone(tiz);
      tkIz = tiz;
    }
    if ((tff != tkFF)) {
      topPid.setFF(tff);
      tkFF = tff;
    }
    if ((tmax != tkMaxOutput) || (tmin != tkMinOutput)) {
      topPid.setOutputRange(tmin, tmax);
      tkMinOutput = tmin;
      tkMaxOutput = tmax;
    }

    // TODO: probably remove bottom since shooter will have one motor, not two independent motors
    //
    // if ((bp != bkP)) {
    //   bottomPid.setP(bp);
    //   bkP = bp;
    // }
    // if ((bi != bkI)) {
    //   bottomPid.setI(bi);
    //   bkI = bi;
    // }
    // if ((bd != bkD)) {
    //   bottomPid.setD(bd);
    //   bkD = bd;
    // }
    // if ((biz != bkIz)) {
    //   bottomPid.setIZone(biz);
    //   bkIz = biz;
    // }
    // if ((bff != bkFF)) {
    //   bottomPid.setFF(bff);
    //   bkFF = bff;
    // }
    // if ((bmax != bkMaxOutput) || (bmin != bkMinOutput)) {
    //   bottomPid.setOutputRange(bmin, bmax);
    //   bkMinOutput = bmin;
    //   bkMaxOutput = bmax;
    // }
  }

  @Override
  public void setVelocity(double velocityRadPerSec, double ffVolts) {

    topPid.setReference(
        Units.radiansPerSecondToRotationsPerMinute(velocityRadPerSec) * GEAR_RATIO,
        CANSparkMax.ControlType.kVelocity,
        0,
        ffVolts,
        ArbFFUnits.kVoltage);

    SmartDashboard.putNumber("Shooter/top/velocityRadPerSec", velocityRadPerSec);
    SmartDashboard.putNumber("Shooter/top/ProcessVariable", topEncoder.getVelocity());

    // TODO: probably remove bottom since shooter will have one motor, not two independent motors
    //
    // bottomPid.setReference(setPoint, CANSparkMax.ControlType.kVelocity);
    // SmartDashboard.putNumber("Shooter/bot/SetPoint", setPoint);
    // SmartDashboard.putNumber("Shooter/bot/ProcessVariable", bottomEncoder.getVelocity());
  }

  @Override
  public void setVoltage(double volts) {
    // Set the voltage output for the top motor
    top.setVoltage(volts);
    bottom.setVoltage(volts);
  }
}
