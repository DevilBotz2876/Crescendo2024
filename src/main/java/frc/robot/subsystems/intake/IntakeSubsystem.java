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

  // Create a Mechanism2d display of an Intake
  private final Mechanism2d mech2d = new Mechanism2d(60, 60);
  private final MechanismRoot2d intakePivot2d = mech2d.getRoot("IntakePivot", 45, 30);
  private List<MechanismLigament2d> intake2d = new ArrayList<MechanismLigament2d>();
  private final MechanismRoot2d notePivot2d = mech2d.getRoot("NotePivot", 15, 30);
  private List<MechanismLigament2d> note2d = new ArrayList<MechanismLigament2d>();
  private boolean noteVisibility = false;

  private int currentSimAngle = 0;

  public IntakeSubsystem(IntakeIO IO) {
    this.IO = IO;

    targetVoltage = 0;

    intake2d.add(
        intakePivot2d.append(
            new MechanismLigament2d("WheelA", 10, 0, 6, new Color8Bit(Color.kGray))));
    intake2d.add(
        intakePivot2d.append(
            new MechanismLigament2d("WheelB", 10, 90, 6, new Color8Bit(Color.kRed))));
    intake2d.add(
        intakePivot2d.append(
            new MechanismLigament2d("WheelC", 10, 180, 6, new Color8Bit(Color.kGray))));
    intake2d.add(
        intakePivot2d.append(
            new MechanismLigament2d("WheelD", 10, 270, 6, new Color8Bit(Color.kRed))));

    note2d.add(
        notePivot2d.append(
            new MechanismLigament2d("NoteA", 0, 0, 0, new Color8Bit(Color.kOrange))));

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
      if (targetVoltage < 0) {
        currentSimAngle += 30;
      } else if (targetVoltage > 0) {
        currentSimAngle -= 30;
      }

      int angleOffset = 0;
      for (MechanismLigament2d intake : intake2d) {
        intake.setAngle(angleOffset + currentSimAngle);
        angleOffset += 90;
      }
    }

    if (inputs.limitSwitchShooter != noteVisibility) {
      noteVisibility = inputs.limitSwitchShooter;

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

  public boolean isPieceDetected(boolean intakePieceDetection) {
    if (intakePieceDetection == true) {
      return inputs.limitSwitchIntake;
    } else {
      return inputs.limitSwitchShooter;
    }
  }
}
