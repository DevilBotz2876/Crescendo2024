package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController.ArbFFUnits;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.config.RobotConfig.ShooterConstants;

public class ShooterIOSparkMax implements ShooterIO {
  // Gear ratio for the shooter mechanism
  private static final double GEAR_RATIO = 1.0;

  private final CANSparkMax flywheel;
  private SparkPIDController pid;

  public double tkP, tkI, tkD, tkIz, tkFF, tkMaxOutput, tkMinOutput, tmaxRPS;

  // Gets the NEO encoder
  private final RelativeEncoder encoder;

  public ShooterIOSparkMax(int id) {
    flywheel = new CANSparkMax(id, MotorType.kBrushless);
    flywheel.restoreFactoryDefaults();

    flywheel.setInverted(false);
    flywheel.enableVoltageCompensation(10.0);
    flywheel.setSmartCurrentLimit(30);
    // Set motor to brake mode so shooter stops spinning immediately
    flywheel.setIdleMode(IdleMode.kBrake);

    encoder = flywheel.getEncoder();

    // Short answer to why we need to change these values.  It's a good starting point to try
    // period=16 and depth=2
    // https://www.chiefdelphi.com/t/what-is-the-best-performance-for-a-shooter/457903/25
    //
    // Read this entire thread to get complete story, this specific post again mentions values to
    // use for period and depth that worked for one team.
    // https://www.chiefdelphi.com/t/psa-default-neo-sparkmax-velocity-readings-are-still-bad-for-flywheels/454453/32
    REVLibError revStatus;
    int periodMS = 16;
    revStatus = encoder.setMeasurementPeriod(periodMS);
    if (revStatus != REVLibError.kOk) {
      System.out.println(
          "Failed to set shooter encoder measurement period to "
              + periodMS
              + " revStatus="
              + revStatus);
    }
    periodMS = encoder.getMeasurementPeriod();
    System.out.println("shooter encoder measurement period set to " + periodMS);

    int avgDepth = 2;
    revStatus = encoder.setAverageDepth(avgDepth);
    if (revStatus != REVLibError.kOk) {
      System.out.println(
          "Failed to set shooter encoder average depth to " + avgDepth + " revStatus=" + revStatus);
    }
    avgDepth = encoder.getAverageDepth();
    System.out.println("shooter encoder average depth to " + avgDepth);

    // Tips for tuning flywheel.  Use feed-forward AND PID.  Don't use I/D terms in PID, just P.
    //
    // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/tuning-flywheel.html#choice-of-control-strategies

    // tkP = 0.0010514;
    tkP = ShooterConstants.pidKp;
    tkI = 0;
    tkD = 0;
    tkIz = 0;
    // tkFF = 0.08134;
    tkFF = ShooterConstants.ffKv;
    tkMaxOutput = 1;
    tkMinOutput = -1;
    tmaxRPS = 300;

    pid = flywheel.getPIDController();
    pid.setP(tkP);
    pid.setI(tkI);
    pid.setD(tkD);
    pid.setIZone(tkIz);
    pid.setFF(tkFF);
    pid.setOutputRange(tkMinOutput, tkMaxOutput);

    // TODO: probably remove bottom since shooter will have one motor, not two independent motor
    if (Constants.debugMode) {
      SmartDashboard.putNumber("Shooter/top/P Gain", tkP);
      SmartDashboard.putNumber("Shooter/top/I Gain", tkI);
      SmartDashboard.putNumber("Shooter/top/D Gain", tkD);
      SmartDashboard.putNumber("Shooter/top/I Zone", tkIz);
      SmartDashboard.putNumber("Shooter/top/Feed Forward", tkFF);
      SmartDashboard.putNumber("Shooter/top/Max Output", tkMaxOutput);
      SmartDashboard.putNumber("Shooter/top/Min Output", tkMinOutput);
    }
    // Last thing we do is save all settings to flash on sparkmax
    flywheel.burnFlash();
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    // Set velocityRadPerSec to the encoder velocity(rotationsPerMinute) divided by the gear ratio
    // and converted into Radians Per Second
    inputs.velocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity() / GEAR_RATIO);
    inputs.appliedVolts = flywheel.getAppliedOutput() * flywheel.getBusVoltage();
    inputs.current = flywheel.getOutputCurrent();

    // double tp = SmartDashboard.getNumber("Shooter/P Gain", 0);
    // double ti = SmartDashboard.getNumber("Shooter/I Gain", 0);
    // double td = SmartDashboard.getNumber("Shooter/D Gain", 0);
    // double tiz = SmartDashboard.getNumber("Shooter/I Zone", 0);
    // double tff = SmartDashboard.getNumber("Shooter/Feed Forward", 0);
    // double tmax = SmartDashboard.getNumber("Shooter/Max Output", 0);
    // double tmin = SmartDashboard.getNumber("Shooter/Min Output", 0);

    // if ((tp != tkP)) {
    //   pid.setP(tp);
    //   tkP = tp;
    // }
    // if ((ti != tkI)) {
    //   pid.setI(ti);
    //   tkI = ti;
    // }
    // if ((td != tkD)) {
    //   pid.setD(td);
    //   tkD = td;
    // }
    // if ((tiz != tkIz)) {
    //   pid.setIZone(tiz);
    //   tkIz = tiz;
    // }
    // if ((tff != tkFF)) {
    //   pid.setFF(tff);
    //   tkFF = tff;
    // }
    // if ((tmax != tkMaxOutput) || (tmin != tkMinOutput)) {
    //   pid.setOutputRange(tmin, tmax);
    //   tkMinOutput = tmin;
    //   tkMaxOutput = tmax;
    // }

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

    if (Constants.debugMode) {
      SmartDashboard.putNumber("Shooter/top/velocityRadPerSec", velocityRadPerSec);
      SmartDashboard.putNumber("Shooter/top/ProcessVariable", encoder.getVelocity());
    }

    // TODO: probably remove bottom since shooter will have one motor, not two independent motors
    //

  }

  public boolean supportsHardwarePid() {
    return false;
  }

  @Override
  public void setVoltage(double volts) {
    flywheel.setVoltage(volts);
  }
}
