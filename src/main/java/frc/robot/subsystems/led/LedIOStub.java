package frc.robot.subsystems.led;

public class LedIOStub implements LedIO {

  int[] RGB = {0, 0, 0};

  public LedIOStub() {}

  @Override
  public void updateInputs(LedIOInputs inputs) {
    inputs.red = RGB[0];
    inputs.green = RGB[1];
    inputs.blue = RGB[2];
  }

  @Override
  public void setColor(int red, int green, int blue) {
    RGB[0] = red;
    RGB[1] = green;
    RGB[2] = blue;
  }
}
