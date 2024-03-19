package frc.robot.subsystems.led;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LedSystem extends SubsystemBase implements Led {
  private final LedIO io;
  private final LedIOInputsAutoLogged inputs = new LedIOInputsAutoLogged();
  private List<MechanismLigament2d> led2d = new ArrayList<MechanismLigament2d>();

  public LedSystem(LedIO io) {
    this.io = io;
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
    MechanismRoot2d LedPivot2d = mech2d.getRoot("LED Pivot", 50, 15);
    
    led2d.add(
        LedPivot2d.append(new MechanismLigament2d("LED", 5, 0, 10, new Color8Bit(Color.kOrange))));
  }
  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Led", inputs);


  }
}
