package frc.robot.subsystems.arm;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.simulation.DutyCycleEncoderSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import org.littletonrobotics.junction.AutoLogOutput;

public class ArmIOStub implements ArmIO {
  // The P gain for the PID controller that drives this arm.
  private double armKp = .1;
  private double armKd = 0;
  @AutoLogOutput private double targetDegrees = 0;
  @AutoLogOutput private double feedForwardVolts = 0;
  private double armAbsoluteOffset = 0.0;
  private double armGearingReduction = 317;
  private double armLengthInMeters = .5;
  private double minAngleInDegrees = 0;
  private double maxAngleInDegrees = 103;
  private double armMassInKg = 11.3398;

  // The arm gearbox represents a gearbox containing two Vex 775pro motors.
  private final DCMotor motorPlant = DCMotor.getNEO(1);

  // Standard classes for controlling our arm
  private final PIDController pid = new PIDController(armKp, 0, armKd);
  private final DutyCycleEncoder absEncoder = new DutyCycleEncoder(0);
  private final DutyCycleEncoderSim absEncoderSim = new DutyCycleEncoderSim(absEncoder);
  private final Encoder relEncoder = new Encoder(1, 2);
  private final EncoderSim relEncoderSim = new EncoderSim(relEncoder);
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
          Units.degreesToRadians(minAngleInDegrees),
          Units.degreesToRadians(maxAngleInDegrees),
          true,
          1.0,
          VecBuilder.fill(0));

  public ArmIOStub() {
    absEncoder.setPositionOffset(armAbsoluteOffset);
    absEncoder.setDutyCycleRange(1.0 / 1025.0, 1024.0 / 1025.0);
    absEncoder.setDistancePerRotation(360.0); 
  }

  /** Updates the set of loggable inputs. */
  @Override
  public void updateInputs(ArmIOInputs inputs) {
    inputs.positionDegree = absEncoder.getDistance();
    inputs.positionRad = Units.degreesToRadians(absEncoder.getDistance());
    inputs.leftAppliedVolts = motor.get() * RobotController.getBatteryVoltage();

    arm.setInput(inputs.leftAppliedVolts);
    arm.update(0.020);
    absEncoderSim.setDistance(Units.radiansToDegrees(arm.getAngleRads()));

    if (softwarePidEnabled) {
      motor.setVoltage(
          feedForwardVolts
              + pid.calculate(absEncoder.getDistance(), targetDegrees)
                  * RobotController.getBatteryVoltage());
    }
  }

  /** Run the arm motor at the specified voltage. */
  @Override
  public void setVoltage(double volts) {
    motor.setVoltage(volts);
  }

  @Override
  public void setPosition(double degrees, double ffVolts) {
    targetDegrees = degrees;
    feedForwardVolts = ffVolts;
    softwarePidEnabled = true;
  }

  @Override
  public void setFeedback(double kP, double kI, double kD) {
    pid.setP(kP);
    pid.setI(kI);
    pid.setD(kD);
  }

  @Override
  public void resetRelativeEncoder(double position) {
    relEncoderSim.setDistance(position);
  }
}
