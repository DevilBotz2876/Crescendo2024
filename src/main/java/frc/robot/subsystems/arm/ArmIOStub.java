package frc.robot.subsystems.arm;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.config.RobotConfig;
import frc.robot.config.RobotConfig.ArmConstants;

public class ArmIOStub implements ArmIO {
  // The P gain for the PID controller that drives this arm.
  private double targetDegrees = 0;
  private double feedForwardVolts = 0;
  private double armGearingReduction = 317;
  private double armLengthInMeters = .5;
  private double minAngleInDegrees = ArmConstants.minAngleInDegrees;
  private double maxAngleInDegrees = ArmConstants.maxAngleInDegrees;
  private double armMassInKg = 11.3398;

  // The arm gearbox represents a gearbox containing two Vex 775pro motors.
  private final DCMotor motorPlant = DCMotor.getNEO(1);
  private double currentVoltage = 0.0;

  // Standard classes for controlling our arm
  private final PIDController pid =
      new PIDController(
          RobotConfig.ArmConstants.pidKp,
          RobotConfig.ArmConstants.pidKi,
          RobotConfig.ArmConstants.pidKd);
  private boolean softwarePidEnabled = false;

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
    inputs.positionRad = arm.getAngleRads();
    inputs.positionDegree = Units.radiansToDegrees(inputs.positionRad);
    inputs.velocityInDegrees = Units.radiansToDegrees(arm.getVelocityRadPerSec());
    inputs.appliedVolts = currentVoltage;

    if (softwarePidEnabled) {
      currentVoltage =
          feedForwardVolts
              + pid.calculate(inputs.positionDegree, targetDegrees)
                  * RobotController.getBatteryVoltage();
    }

    arm.setInput(currentVoltage);
    arm.update(0.020);
  }

  /** Run the arm motor at the specified voltage. */
  @Override
  public void setVoltage(double volts) {
    softwarePidEnabled = false;
    currentVoltage = volts;
  }

  @Override
  public void setPosition(double degrees, double ffVolts) {
    targetDegrees = degrees;
    feedForwardVolts = ffVolts;
    softwarePidEnabled = true;
    pid.reset();
  }

  @Override
  public void setFeedback(double kP, double kI, double kD, double minOutput, double maxOutput) {
    pid.setP(kP);
    pid.setI(kI);
    pid.setD(kD);
    // wpilib pid has no method to set output range. When you call calculate method to use output
    // you can limit the range there.
  }
}
