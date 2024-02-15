package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.RobotController;

public class IntakeIOTalonSRX implements IntakeIO {
  //  private static final double GEAR_RATIO = 10.0;

  // define the 1 SparkMax Controller
  private final TalonSRX leader = new TalonSRX(3);

  public IntakeIOTalonSRX() {
    // leader motor is not inverted, and set follower motor to follow the leader
    leader.setInverted(false);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    // Set velocityRadPerSec to the encoder velocity(rotationsPerMinute) divided by the gear ratio
    // and converted into Radians Per Second
    //  inputs.velocityRadPerSec =
    //      Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity() / GEAR_RATIO);
    // Get applied voltage from the leader motor
    //    inputs.appliedVolts = leader.getAppliedOutput() * leader.getBusVoltage();
  }

  @Override
  public void setVoltage(double volts) {
    // Set the voltage output for the leader motor
    leader.set(ControlMode.PercentOutput, volts / RobotController.getBatteryVoltage());
  }
}
