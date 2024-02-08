package frc.robot.subsystems.arm;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.simulation.DutyCycleEncoderSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import org.littletonrobotics.junction.AutoLogOutput;

public class ArmIOStub implements ArmIO {
  // The P gain for the PID controller that drives this arm.
  private double armKp = .1;
  private double armKd = 0;
  @AutoLogOutput private double targetRadians = 0;
  @AutoLogOutput private double feedForwardVolts = 0;
  private double armAbsoluteOffset = 0.0;
  private double armGearingReduction = 30;
  private double armLengthInMeters = .5;
  private double minAngleInRadians = 0;
  private double maxAngleInRadians = 2*Math.PI;
  private double armMassInKg = 8.0;

  // The arm gearbox represents a gearbox containing two Vex 775pro motors.
  private final DCMotor motorPlant = DCMotor.getNEO(1);

  // Standard classes for controlling our arm
  private final PIDController pid = new PIDController(armKp, 0, armKd);
  private final DutyCycleEncoder encoder = new DutyCycleEncoder(0);
  private final DutyCycleEncoderSim encoderSim = new DutyCycleEncoderSim(encoder);
  private final Spark motor = new Spark(0);
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
          minAngleInRadians,
          maxAngleInRadians,
          true,
          0,
          VecBuilder.fill(0));

  public ArmIOStub() {
    encoder.setPositionOffset(armAbsoluteOffset);
    encoder.setDutyCycleRange(1.0 / 1025.0, 1024.0 / 1025.0);
    encoder.setDistancePerRotation(2.0 * Math.PI);
  }

  /** Updates the set of loggable inputs. */
  @Override
  public void updateInputs(ArmIOInputs inputs) {
    inputs.positionRad = encoder.getDistance();
    inputs.leftAppliedVolts = motor.get() * RobotController.getBatteryVoltage();

    arm.setInput(inputs.leftAppliedVolts);
    arm.update(0.020);
    encoderSim.setDistance(arm.getAngleRads());

    if (softwarePidEnabled)
    {
      motor.setVoltage(
          feedForwardVolts
              + pid.calculate(encoder.getDistance(), targetRadians)
                  * RobotController.getBatteryVoltage());
    }
  }

  /** Run the arm motor at the specified voltage. */
  @Override
  public void setVoltage(double volts) {
    motor.setVoltage(volts);
  }

  @Override
  public void setPosition(double radians, double ffVolts) {
    targetRadians = radians;
    feedForwardVolts = ffVolts;
    softwarePidEnabled = true;
  }
}
