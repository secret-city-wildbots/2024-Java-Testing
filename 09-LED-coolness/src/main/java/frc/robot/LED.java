package frc.robot;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Utility.LEDHelpers;

public class LED {
    private final AddressableLED m_led = new AddressableLED(9);
    private final AddressableLEDBuffer m_ledBuffer;

    enum LEDStates {
        NORMAL,
        PARTY,
        LOCATE,
        TEST
    }

    private double chaserStatus = 0;

    private LEDStates ledState = LEDStates.NORMAL; // default to normal

    public LED(int numberOfLEDs) {
        m_ledBuffer = new AddressableLEDBuffer(numberOfLEDs);
    }

    public void updateLED(XboxController driverController) {
        switch (ledState) {
            case NORMAL:

                break;
            case PARTY:
                for (var i = 0; i < m_ledBuffer.getLength(); i++) {
                    var grbVal = LEDHelpers.rgbtogrb(LEDHelpers.hsvToRgb(i * 20 + (int) (chaserStatus), 1, 1));

                    // Sets the specified LED to the HSV values
                    m_ledBuffer.setRGB(i, i * 20 + (int) (grbVal.substring(0,1)), 100, 100);
                }

                // Set the data
                m_led.setData(m_ledBuffer);

                chaserStatus += 2;
                chaserStatus %= 360;
                break;
            case LOCATE:

                break;
            case TEST:

                break;
        }
    }

}
