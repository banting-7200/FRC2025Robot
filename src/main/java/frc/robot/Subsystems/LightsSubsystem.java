// skib fein rizz autism
package frc.robot.Subsystems;

// imports //
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class LightsSubsystem {
  private AddressableLED ledInstance;
  private AddressableLEDBuffer bufferInstance;
  private int rainbowFirstPixelHue = 0;

  boolean areLightsOn = false;

  public LightsSubsystem(int lightPort, int lightCount) {
    ledInstance = new AddressableLED(lightPort);
    bufferInstance = new AddressableLEDBuffer(lightCount);
    ledInstance.setLength(lightCount);
    ledInstance.start();
  }

  public void run() {
    ledInstance.setData(bufferInstance);
  }

  public void solidColor(int red, int green, int blue) {
    for (int x = 0; x < bufferInstance.getLength(); x++) {
      bufferInstance.setRGB(x, red, green, blue);
    }
  }

  public void hasCoral() {
    for (int x = 0; x < bufferInstance.getLength(); x++) {
      bufferInstance.setRGB(x, 255, 255, 255); // white
    }
  }

  public void hasAlgae() {
    for (int x = 0; x < bufferInstance.getLength(); x++) {
      bufferInstance.setRGB(x, 43, 78, 219); // blue
    }
  }

  public void hasNothing() {
    for (int x = 0; x < bufferInstance.getLength(); x++) {
      bufferInstance.setRGB(x, 199, 97, 225); // pink
    }
  }
}
