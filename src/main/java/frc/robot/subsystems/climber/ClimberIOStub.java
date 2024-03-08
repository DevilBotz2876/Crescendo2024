package frc.robot.subsystems.climber;

import frc.robot.config.RobotConfig.ClimberConstants;

public class ClimberIOStub implements ClimberIO {
  private double voltage = 0.0;
  private double position = 0.0;
  private double velocityRadiansPerSec = 1;

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    if (voltage > 0) {
      inputs.velocityRadiansPerSecond = velocityRadiansPerSec;
    } else if (voltage < 0) {
      inputs.velocityRadiansPerSecond = -velocityRadiansPerSec;
    } else {
      inputs.velocityRadiansPerSecond = 0;
    }

    inputs.appliedVolts = voltage;

    if (voltage != 0) {
      if ((position >= -0.5) || (voltage > 0)) {
        // We aren't passed the absolute min limits for a negative voltage or we are trying to move
        // up
        position += inputs.velocityRadiansPerSecond / 50;
        inputs.current = ClimberConstants.autoZeroMaxCurrent / 2;
      } else if (voltage < 0) {
        // We are passed the absolute min limit and we are still trying to move down, so don't move
        // the climber, clamp the voltage and spike the current instead
        inputs.current = ClimberConstants.autoZeroMaxCurrent + 1;
        inputs.appliedVolts = voltage / 4;
        inputs.velocityRadiansPerSecond = 0;
      }
    } else {
      inputs.current = 0;
    }

    inputs.positionRadians = position;
  }

  @Override
  public void setVoltage(double volts) {
    this.voltage = volts;
  }

  @Override
  public void setPosition(double position) {
    this.position = position;
  }
}
