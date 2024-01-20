package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.util.Units;

public class ShooterIOSparkMax implements ShooterIO {
  private static final double GEAR_RATIO = 5.0;

  private final CANSparkMax leader = new CANSparkMax(0, MotorType.kBrushless);
  private final CANSparkMax follower = new CANSparkMax(1, MotorType.kBrushless);
  private final RelativeEncoder encoder = leader.getEncoder();

  public ShooterIOSparkMax() {
    leader.setInverted(false);
    follower.follow(leader, false);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.velocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity() / GEAR_RATIO);
    inputs.appliedVolts = leader.getAppliedOutput() * leader.getBusVoltage();
  }

  @Override
  public void setVoltage(double volts) {
    leader.setVoltage(volts);
  }
}
