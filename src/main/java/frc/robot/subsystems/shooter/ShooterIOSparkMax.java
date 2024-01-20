package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.util.Units;

public class ShooterIOSparkMax implements ShooterIO {
  // Gear ratio for the shooter mechanism
  private static final double GEAR_RATIO = 5.0;

  //define the 2 SparkMax Controllers. A leader, and a follower
  private final CANSparkMax leader = new CANSparkMax(0, MotorType.kBrushless);
  private final CANSparkMax follower = new CANSparkMax(1, MotorType.kBrushless);

  //Gets the NEO encoder
  private final RelativeEncoder encoder = leader.getEncoder();

  public ShooterIOSparkMax() {
     // leader motor is not inverted, and set follower motor to follow the leader
    leader.setInverted(false);
    follower.follow(leader, false);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    // Set velocityRadPerSec to the encoder velocity(rotationsPerMinute) divided by the gear ratio and converted into Radians Per Second
    inputs.velocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity() / GEAR_RATIO);
    // Get applied voltage from the leader motor
    inputs.appliedVolts = leader.getAppliedOutput() * leader.getBusVoltage();
  }

  @Override
  public void setVoltage(double volts) {
    // Set the voltage output for the leader motor
    leader.setVoltage(volts);
  }
}
