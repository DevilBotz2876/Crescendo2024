package frc.robot.subsystems.intake;

//import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class IntakeIOSim implements IntakeIO{
    private FlywheelSim sim = new FlywheelSim(DCMotor.getNEO(1), 1.5, 0.004);
    private double appliedVolts = 0.0;

    @Override
  public void updateInputs(IntakeIOInputs inputs) {
  

    sim.update(0.02);

    //inputs.positionRad = 0.0;
    inputs.velocityRadPerSec = sim.getAngularVelocityRadPerSec();
    inputs.appliedVolts = appliedVolts;
    //inputs.currentAmps = new double[] {sim.getCurrentDrawAmps()};

   

  }

    @Override
    public void setVoltage(double volts) {
    appliedVolts = volts;
    sim.setInputVoltage(volts);
  }




}

