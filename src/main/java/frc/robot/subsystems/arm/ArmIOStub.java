package frc.robot.subsystems.arm;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.config.RobotConfig.ArmConstants;

public class ArmIOStub implements ArmIO {
  private double minAngleInDegrees = ArmConstants.minAngleInDegrees;
  private double maxAngleInDegrees = ArmConstants.maxAngleInDegrees;

  private double armGearingReduction = 210;
  private double armLengthInMeters = 0.6096;
  private double armMassInKg = 13.60777;
  // The arm gearbox represents a gearbox containing two Vex 775pro motors.
  private final DCMotor motorPlant = DCMotor.getNEO(1);
  private double currentVoltage = 0.0;

  // Simulation classes help us simulate what's going on, including gravity.
  // This arm sim represents an arm that can travel from -75 degrees (rotated down front)
  // to 255 degrees (rotated down in the back).
  private final SingleJointedArmSim arm =
      new SingleJointedArmSim(
          motorPlant,
          armGearingReduction,
          SingleJointedArmSim.estimateMOI(armLengthInMeters, armMassInKg),
          armLengthInMeters,
          Units.degreesToRadians(minAngleInDegrees),
          Units.degreesToRadians(maxAngleInDegrees),
          true,
          Units.degreesToRadians(minAngleInDegrees),
          VecBuilder.fill(0));

  /** Updates the set of loggable inputs. */
  @Override
  public void updateInputs(ArmIOInputs inputs) {
    inputs.positionRads = arm.getAngleRads();
    inputs.velocityRadsPerSecond = arm.getVelocityRadPerSec();
    inputs.positionDegrees = Units.radiansToDegrees(inputs.positionRads);
    inputs.velocityDegreesPerSecond = Units.radiansToDegrees(inputs.velocityRadsPerSecond);
    inputs.relativePositionDegrees = inputs.positionDegrees;
    inputs.appliedVolts = currentVoltage;

    arm.setInput(currentVoltage);
    arm.update(0.020);
  }

  /** Run the arm motor at the specified voltage. */
  @Override
  public void setVoltage(double volts) {
    currentVoltage = volts;
  }
}
