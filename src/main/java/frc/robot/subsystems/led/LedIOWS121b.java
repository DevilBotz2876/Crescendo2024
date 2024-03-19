package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.robot.config.RobotConfig.LedConstants;

public class LedIOWS121b implements LedIO {

  AddressableLED led = new AddressableLED(LedConstants.LedPWDPort);

  AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(60);

  public LedIOWS121b() {
    led.setLength(ledBuffer.getLength());

    led.setData(ledBuffer);
    led.start();
  }
  


  @Override
  public void updateInputs(LedIOInputs inputs) {
    inputs.red = ledBuffer.getRed(0);
    inputs.blue = ledBuffer.getGreen(0);
    inputs.green = ledBuffer.getGreen(0);
  }

  @Override
  public void setColor(int red, int green, int blue) {
    for (var i = 0; i < ledBuffer.getLength(); i++) {
        // Sets the specified LED to the RGB values for red
        ledBuffer.setRGB(i, red, green, blue);
     }
     
     led.setData(ledBuffer);
     
  }
}
