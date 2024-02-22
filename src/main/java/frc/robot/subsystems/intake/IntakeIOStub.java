package frc.robot.subsystems.intake;

// import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class IntakeIOStub implements IntakeIO {
  private FlywheelSim sim = new FlywheelSim(DCMotor.getNEO(1), 1.5, 0.004);
  private double appliedVolts = 0.0;
  private boolean limitSwitchIntakeToggled = false;
  private boolean limitSwitchShooterToggled = false;
  private double elapsedTimeMotorOn = 0.0;

  @Override
  public void updateInputs(IntakeIOInputs inputs) {

    sim.update(0.02);

    // inputs.positionRad = 0.0;
    inputs.velocityRadPerSec = sim.getAngularVelocityRadPerSec();
    inputs.appliedVolts = appliedVolts;
    // inputs.currentAmps = new double[] {sim.getCurrentDrawAmps()};

    // Simulate intake/shooter limit switches
    if (appliedVolts != 0.0) {
      elapsedTimeMotorOn += 0.02;

      if (!limitSwitchIntakeToggled
          && (elapsedTimeMotorOn > (inputs.limitSwitchIntake ? 0.5 : 3.0))) {
        inputs.limitSwitchIntake = !inputs.limitSwitchIntake;
        limitSwitchIntakeToggled = true;
      }

      if (!limitSwitchShooterToggled
          && (elapsedTimeMotorOn > (inputs.limitSwitchShooter ? 1.0 : 2.0))) {
        inputs.limitSwitchShooter = !inputs.limitSwitchShooter;
        limitSwitchShooterToggled = true;
      }
    } else {
      elapsedTimeMotorOn = 0.0;
      limitSwitchIntakeToggled = false;
      limitSwitchShooterToggled = false;
    }
  }

  @Override
  public void setVoltage(double volts) {
    appliedVolts = volts;
    sim.setInputVoltage(volts);
  }
}
