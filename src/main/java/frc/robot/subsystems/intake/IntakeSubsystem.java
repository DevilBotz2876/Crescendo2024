package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem extends SubsystemBase implements Intake {
  private final IntakeIO IO;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  @AutoLogOutput private double targetVoltage;

  public IntakeSubsystem(IntakeIO IO) {
    this.IO = IO;

    targetVoltage = 0;
  }

  @Override
  public void runVoltage(double volts) {
    targetVoltage = volts;
  }

  @Override
  public void periodic() {
    IO.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);

    IO.setVoltage(targetVoltage);
  }

  public boolean isPieceDetected(boolean intakePieceDetection) {
    if (intakePieceDetection == true) {
      return inputs.limitSwitchIntake;
    } else {
      return inputs.limitSwitchShooter;
    }
  }
}
