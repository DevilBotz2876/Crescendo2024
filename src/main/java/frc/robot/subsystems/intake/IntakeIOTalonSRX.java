package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;

public class IntakeIOTalonSRX implements IntakeIO {
  //  private static final double GEAR_RATIO = 10.0;

  // define the 1 SparkMax Controller
  private final TalonSRX leader;
  DigitalInput limitSwitchIntake = new DigitalInput(1);
  DigitalInput limitSwitchShooter = new DigitalInput(2);

  public IntakeIOTalonSRX(int id) {
    leader = new TalonSRX(id);
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
    inputs.appliedVolts = leader.getMotorOutputVoltage();
    inputs.limitSwitchIntake = !limitSwitchIntake.get();
    inputs.limitSwitchShooter = !limitSwitchShooter.get();
  }

  @Override
  public void setVoltage(double volts) {
    // Set the voltage output for the leader motor
    leader.set(ControlMode.PercentOutput, volts / RobotController.getBatteryVoltage());
  }
}
