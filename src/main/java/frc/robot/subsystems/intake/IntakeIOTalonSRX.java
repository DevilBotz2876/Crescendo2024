package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;

public class IntakeIOTalonSRX implements IntakeIO {
  //  private static final double GEAR_RATIO = 10.0;

  // define the 1 SparkMax Controller
  private final TalonSRX leader;
  DigitalInput limitSwitchIntake = new DigitalInput(1);

  public IntakeIOTalonSRX(int id, boolean inverted) {
    leader = new TalonSRX(id);
    // leader motor is not inverted, and set follower motor to follow the leader
    leader.setInverted(inverted);
    leader.setNeutralMode(NeutralMode.Brake);

    /* Peak Current and Duration must be exceeded before current limit is activated.
    When activated, current will be limited to Continuous Current.
    Set Peak Current params to 0 if desired behavior is to immediately current-limit. */
    leader.configPeakCurrentLimit(35, 10); /* 35 A */
    leader.configPeakCurrentDuration(200, 10); /* 200ms */
    leader.configContinuousCurrentLimit(30, 10); /* 30 */
    leader.enableCurrentLimit(true);
  }

  public IntakeIOTalonSRX(int id) {
    this(id, false);
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
    inputs.current = leader.getStatorCurrent();
  }

  @Override
  public void setVoltage(double volts) {
    // Set the voltage output for the leader motor
    leader.set(ControlMode.PercentOutput, volts / RobotController.getBatteryVoltage());
  }
}
