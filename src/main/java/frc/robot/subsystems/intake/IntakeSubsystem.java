package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem extends SubsystemBase implements Intake {
  private final IntakeIO IO;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  @AutoLogOutput private double targetVoltage;

  // Mechanism2d display of an Intake
  private List<MechanismLigament2d> intake2d = new ArrayList<MechanismLigament2d>();
  private List<MechanismLigament2d> note2d = new ArrayList<MechanismLigament2d>();
  private boolean noteVisibility = false;

  private int currentSimAngle = 0;

  public IntakeSubsystem(IntakeIO IO) {
    this.IO = IO;

    targetVoltage = 0;

    // Create 2D simulated display of an Intake/Note
    Mechanism2d mech2d = new Mechanism2d(60, 60);
    MechanismRoot2d intakePivot2d = mech2d.getRoot("Intake Pivot", 50, 50);
    MechanismRoot2d notePivot2d = mech2d.getRoot("Note Pivot", 25, 40);

    intake2d.add(
        intakePivot2d.append(
            new MechanismLigament2d("Wheel Spoke A", 5, 0, 6, new Color8Bit(Color.kGray))));
    intake2d.add(
        intakePivot2d.append(
            new MechanismLigament2d("Wheel Spoke B", 5, 90, 6, new Color8Bit(Color.kRed))));
    intake2d.add(
        intakePivot2d.append(
            new MechanismLigament2d("Wheel Spoke C", 5, 180, 6, new Color8Bit(Color.kGray))));
    intake2d.add(
        intakePivot2d.append(
            new MechanismLigament2d("Wheel Spoke D", 5, 270, 6, new Color8Bit(Color.kRed))));

    note2d.add(
        notePivot2d.append(new MechanismLigament2d("Note", 0, 0, 0, new Color8Bit(Color.kOrange))));

    SmartDashboard.putData("Intake Simulation", mech2d);
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

    if (targetVoltage != 0) {
      currentSimAngle -= (targetVoltage / 12) * 30;

      int angleOffset = 0;
      for (MechanismLigament2d intake : intake2d) {
        intake.setAngle(angleOffset + currentSimAngle);
        angleOffset += 90;
      }
    }

    if (inputs.limitSwitchIntake != noteVisibility) {
      noteVisibility = inputs.limitSwitchIntake;

      for (MechanismLigament2d note : note2d) {
        if (noteVisibility) {
          note.setLength(10);
          note.setLineWeight(10);
        } else {
          note.setLength(0);
          note.setLineWeight(0);
        }
      }
    }
  }

  public boolean isPieceDetected() {
    return inputs.limitSwitchIntake;
  }
}
