package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.RobotConfig.IntakeConstants;
import java.util.Map;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class IntakeBase extends SubsystemBase implements Intake {
  private final IntakeIO IO;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  @AutoLogOutput private double targetVoltage;

  public IntakeBase(IntakeIO IO) {
    this.IO = IO;

    targetVoltage = 0;

    ShuffleboardTab assistTab = Shuffleboard.getTab("Assist");
    // Create volt entry under Shooter tab as a number sider with min = -1 and max = 1
    assistTab
        .add("Intake Piece Volts", IntakeConstants.intakeSpeedInVolts)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", 0, "max", 12));

    assistTab
        .add("Index Piece Volts", IntakeConstants.indexSpeedInVolts)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", 0, "max", 12));

    assistTab
        .add("Feed Piece Volts", IntakeConstants.feedSpeedInVolts)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", 0, "max", 12));
  }

  @Override
  public void setVoltage(double volts) {
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
