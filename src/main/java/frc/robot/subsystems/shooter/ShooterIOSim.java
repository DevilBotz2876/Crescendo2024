package frc.robot.subsystems.shooter;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class ShooterIOSim implements ShooterIO {
  private FlywheelSim wheel;

  private double appliedVolts = 0.0;

  public enum ShooterId {
    SHOOTER_TOP,
    SHOOTER_BOTTOM
  }

  public ShooterIOSim() {
    this(ShooterId.SHOOTER_TOP);
  }

  public ShooterIOSim(ShooterId id) {
    switch (id) {
      case SHOOTER_TOP:
        wheel = new FlywheelSim(DCMotor.getNEO(1), 1.5, 0.004);
        break;
      case SHOOTER_BOTTOM:
        wheel = new FlywheelSim(DCMotor.getNEO(1), 1.5, 0.006);
        break;
    }
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {

    // Update sim
    wheel.update(0.02);

    // Update inputs
    inputs.velocityRadPerSec = wheel.getAngularVelocityRadPerSec();
    inputs.appliedVolts = appliedVolts;
  }

  @Override
  public void setVoltage(double volts) {
    appliedVolts = volts;
    wheel.setInputVoltage(volts);
  }
}
