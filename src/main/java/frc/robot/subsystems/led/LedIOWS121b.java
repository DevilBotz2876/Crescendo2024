package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.robot.config.RobotConfig.LedConstants;

public class LedIOWS121b implements LedIO {

  AddressableLED led1 = new AddressableLED(LedConstants.Led1PWDPort);

  AddressableLEDBuffer led1Buffer = new AddressableLEDBuffer(LedConstants.Led1Length);

  public LedIOWS121b() {
    led1.setLength(led1Buffer.getLength());

    led1.setData(led1Buffer);
    led1.start();
  }

  @Override
  public void updateInputs(LedIOInputs inputs) {
    inputs.red = led1Buffer.getRed(0);
    inputs.blue = led1Buffer.getGreen(0);
    inputs.green = led1Buffer.getGreen(0);
  }

  @Override
  public void setColor(int red, int green, int blue) {
    for (var i = 0; i < led1Buffer.getLength(); i++) {
      // Sets the specified LED to the RGB values for red
      led1Buffer.setRGB(i, red, green, blue);
    }

    led1.setData(led1Buffer);
  }
}
