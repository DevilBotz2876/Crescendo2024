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
  private final CANSparkMax flywheel;
  // private final CANSparkMax bottom = new CANSparkMax(1, MotorType.kBrushless);

  private SparkPIDController pid;

  public double tkP, tkI, tkD, tkIz, tkFF, tkMaxOutput, tkMinOutput, tmaxRPS;

  // TODO: probably remove bottom since shooter will have one motor, not two independent motors

  // Gets the NEO encoder
  private final RelativeEncoder encoder;

  public ShooterIOSparkMax(int id) {
    flywheel = new CANSparkMax(id, MotorType.kBrushless);
    pid = flywheel.getPIDController();
    encoder = flywheel.getEncoder();
    flywheel.setInverted(false);

    flywheel.enableVoltageCompensation(12.0);
    flywheel.setSmartCurrentLimit(30);

    flywheel.burnFlash();

    // TODO: these values are samples picked from REV example PID code.  Need to tune PID and choose
    // real values.
    tkP = 0.0010514;
    tkI = 0;
    tkD = 0;
    tkIz = 0;
    tkFF = 0.08134;
    tkMaxOutput = 1;
    tkMinOutput = -1;
    tmaxRPS = 300;

    // TODO: probably remove bottom since shooter will have one motor, not two independent motors

    pid.setP(tkP);
    pid.setI(tkI);
    pid.setD(tkD);
    pid.setIZone(tkIz);
    // pid.setFF(tkFF);
    pid.setOutputRange(tkMinOutput, tkMaxOutput);

    // TODO: probably remove bottom since shooter will have one motor, not two independent motor

    SmartDashboard.putNumber("Shooter/top/P Gain", tkP);
    SmartDashboard.putNumber("Shooter/top/I Gain", tkI);
    SmartDashboard.putNumber("Shooter/top/D Gain", tkD);
    SmartDashboard.putNumber("Shooter/top/I Zone", tkIz);
    SmartDashboard.putNumber("Shooter/top/Feed Forward", tkFF);
    SmartDashboard.putNumber("Shooter/top/Max Output", tkMaxOutput);
    SmartDashboard.putNumber("Shooter/top/Min Output", tkMinOutput);

    // TODO: probably remove this since shooter will have one motor, not two independent motors
    //
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    // Set velocityRadPerSec to the encoder velocity(rotationsPerMinute) divided by the gear ratio
    // and converted into Radians Per Second
    inputs.velocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity() / GEAR_RATIO);
    // Get applied voltage from the top motor
    inputs.appliedVolts = flywheel.getAppliedOutput() * flywheel.getBusVoltage();
    //    inputs.velocityRadPerSecBottom =
    //        Units.rotationsPerMinuteToRadiansPerSecond(bottomEncoder.getVelocity() / GEAR_RATIO);
    // Get applied voltage from the top motor
    //    inputs.appliedVoltsBottom = bottom.getAppliedOutput() * bottom.getBusVoltage();
    inputs.current = flywheel.getOutputCurrent();

    double tp = SmartDashboard.getNumber("Shooter/P Gain", 0);
    double ti = SmartDashboard.getNumber("Shooter/I Gain", 0);
    double td = SmartDashboard.getNumber("Shooter/D Gain", 0);
    double tiz = SmartDashboard.getNumber("Shooter/I Zone", 0);
    double tff = SmartDashboard.getNumber("Shooter/Feed Forward", 0);
    double tmax = SmartDashboard.getNumber("Shooter/Max Output", 0);
    double tmin = SmartDashboard.getNumber("Shooter/Min Output", 0);

    // TODO: probably remove this since shooter will have one motor, not two independent motors
    //

    if ((tp != tkP)) {
      pid.setP(tp);
      tkP = tp;
    }
    if ((ti != tkI)) {
      pid.setI(ti);
      tkI = ti;
    }
    if ((td != tkD)) {
      pid.setD(td);
      tkD = td;
    }
    if ((tiz != tkIz)) {
      pid.setIZone(tiz);
      tkIz = tiz;
    }
    if ((tff != tkFF)) {
      pid.setFF(tff);
      tkFF = tff;
    }
    if ((tmax != tkMaxOutput) || (tmin != tkMinOutput)) {
      pid.setOutputRange(tmin, tmax);
      tkMinOutput = tmin;
      tkMaxOutput = tmax;
    }

    // TODO: probably remove bottom since shooter will have one motor, not two independent motors
    //
  }

  @Override
  public void setVelocity(double velocityRadPerSec, double ffVolts) {

    pid.setReference(
        Units.radiansPerSecondToRotationsPerMinute(velocityRadPerSec) * GEAR_RATIO,
        CANSparkMax.ControlType.kVelocity,
        0,
        ffVolts,
        ArbFFUnits.kVoltage);

    SmartDashboard.putNumber("Shooter/top/velocityRadPerSec", velocityRadPerSec);
    SmartDashboard.putNumber("Shooter/top/ProcessVariable", encoder.getVelocity());

    // TODO: probably remove bottom since shooter will have one motor, not two independent motors
    //

  }

  public boolean supportsHardwarePid() {
    return false;
  }

  @Override
  public void setVoltage(double volts) {
    // Set the voltage output for the top motor
    flywheel.setVoltage(volts);
    // bottom.setVoltage(volts);
  }
}
