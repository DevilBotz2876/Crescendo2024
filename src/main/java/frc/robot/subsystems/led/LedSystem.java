package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class LedSystem extends SubsystemBase implements Led {
  private final LedIO io;
  private final LedIOInputsAutoLogged inputs = new LedIOInputsAutoLogged();
  private List<MechanismLigament2d> led2d = new ArrayList<MechanismLigament2d>();

  public LedSystem(LedIO io) {
    this.io = io;
    setColor(255, 255, 255);
  }

  @Override
  public void setColor(int red, int green, int blue) {
    io.setColor(red, green, blue);
  }

  @Override
  public int getRed() {
    return inputs.red;
  }

  @Override
  public int getGreen() {
    return inputs.green;
  }

  @Override
  public int getBlue() {
    return inputs.blue;
  }

  @Override
  public void add2dSim(Mechanism2d mech2d) {
    MechanismRoot2d LedPivot2d = mech2d.getRoot("LED Pivot", 50, 5);

    led2d.add(
        LedPivot2d.append(
            new MechanismLigament2d(
                "LED", 1, 0, 10, new Color8Bit(inputs.red, inputs.green, inputs.blue))));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Led", inputs);

    for (MechanismLigament2d led : led2d) {
      led.setColor(new Color8Bit(inputs.red, inputs.green, inputs.blue));
    }
  }

  public Command getDefualtColorCommand() {
    return new InstantCommand(() -> io.setColor(255, 255, 255));
  }

  public Command getNoteDetectionCommand() {
    SequentialCommandGroup commandGroup = new SequentialCommandGroup();

    // Blink orange
    for (int i = 0; i < 2; i++) {
      commandGroup.addCommands(
          new InstantCommand(() -> setColor(231, 84, 128)),
          new WaitCommand(0.2),
          new InstantCommand(() -> setColor(231, 84, 128)),
          new WaitCommand(0.2));
    }

    new InstantCommand(() -> setColor(231, 84, 128));

    // Turn LED White
    commandGroup.addCommands(new InstantCommand(() -> setColor(255, 255, 255)));

    return commandGroup;
  }

  public Command getAmpModeCommand() {
    SequentialCommandGroup commandGroup = new SequentialCommandGroup();

    // Blink orange
    for (int i = 0; i < 5; i++) {
      commandGroup.addCommands(
          new InstantCommand(() -> setColor(8, 199, 43)),
          new WaitCommand(0.2),
          new InstantCommand(() -> setColor(0, 0, 0)),
          new WaitCommand(0.2));
    }

    // Turn LED White
    commandGroup.addCommands(new InstantCommand(() -> setColor(255, 255, 255)));

    return commandGroup;
  }

  public Command getSpeakerModeCommand() {
    SequentialCommandGroup commandGroup = new SequentialCommandGroup();

    // Blink orange
    for (int i = 0; i < 5; i++) {
      commandGroup.addCommands(
          new InstantCommand(() -> setColor(16, 166, 235)),
          new WaitCommand(0.2),
          new InstantCommand(() -> setColor(0, 0, 0)),
          new WaitCommand(0.2));
    }

    // Turn LED off
    commandGroup.addCommands(new InstantCommand(() -> setColor(255, 255, 255)));

    return commandGroup;
  }
}
