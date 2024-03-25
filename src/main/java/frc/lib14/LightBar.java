package frc.lib14;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class LightBar {

    private AddressableLED ledStripDriver;
    private AddressableLEDBuffer ledDataBuffer;

    private AddressableLEDBuffer greenDataBuffer;
    private AddressableLEDBuffer redDataBuffer;
    private AddressableLEDBuffer blueDataBuffer;
    private AddressableLEDBuffer yellowDataBuffer;

    public enum Color {
        RED, GREEN, BLUE, YELLOW;
    }

    private LightBar.Color currentColor;

    /**
     * Build and start a new LED LightBar using WS2812 LEDs plugged into the RIO PWM
     * Port.
     * 
     * IMPORTANT: Only one LED Driver is supported per RoboRio at this time.
     *
     * @param portNum the PWM Port Number the WS2812 LED strip is connected to
     * @param length  the lenght of the LED strip as a count of LEDs
     * @see https://docs.wpilib.org/en/latest/docs/software/actuators/addressable-leds.html
     * @see https://first.wpi.edu/FRC/roborio/release/docs/java/edu/wpi/first/wpilibj/AddressableLED.html
     * @see https://first.wpi.edu/FRC/roborio/release/docs/java/edu/wpi/first/wpilibj/AddressableLEDBuffer.html
     */
    public LightBar(int portNum, int length) {
        // Must be a PWM header, not MXP or DIO
        ledStripDriver = new AddressableLED(portNum);

        // Reuse buffer; Length is expensive to set, only set it once, then just update
        // data
        ledDataBuffer = new AddressableLEDBuffer(length);
        ledStripDriver.setLength(ledDataBuffer.getLength());

        // I'm pre-building these color buffers so that we don't have to build them in a
        // loop at runtime.
        greenDataBuffer = buildGreenBuffer(length);
        redDataBuffer = buildRedBuffer(length);
        blueDataBuffer = buildBlueBuffer(length);
        yellowDataBuffer = buildYellowBuffer(length);

        // Set startup da ta
        setColor(Color.GREEN); // default the led array to MCR green
        ledStripDriver.start();
    }

    /**
     * Just run the lightbar and set the LEDs to the current set color
     */
    public void run() {
        switch (this.currentColor) {
        case RED:
            ledStripDriver.setData(redDataBuffer);
            break;
        case GREEN:
            ledStripDriver.setData(greenDataBuffer);
            break;
        case BLUE:
            ledStripDriver.setData(blueDataBuffer);
            break;
        case YELLOW:
            ledStripDriver.setData(yellowDataBuffer);
            break;
        default:
            ledStripDriver.setData(greenDataBuffer);
            break;
        }
    }

    /**
     * Set the color of the lightbar
     * 
     * @param color LightBar.Color of Red, Green, Blue, Yellow
     */
    public void setColor(LightBar.Color color) {
        this.currentColor = color;
    }

    private AddressableLEDBuffer buildGreenBuffer(int length) {
        return buildColorBuffer(new Color8Bit(0, 255, 0), length); // green
    }

    private AddressableLEDBuffer buildRedBuffer(int length) {
        return buildColorBuffer(new Color8Bit(255, 0, 0), length); // red
    }

    private AddressableLEDBuffer buildBlueBuffer(int length) {
        return buildColorBuffer(new Color8Bit(0, 0, 255), length); // blue
    }

    private AddressableLEDBuffer buildYellowBuffer(int length) {
        return buildColorBuffer(new Color8Bit(255, 255, 0), length); // yellow
    }

    private AddressableLEDBuffer buildColorBuffer(Color8Bit color, int length) {
        AddressableLEDBuffer buffer = new AddressableLEDBuffer(length);
        for (var i = 0; i < buffer.getLength(); i++) {
            // Sets the specified LED to the RGB values provided
            buffer.setRGB(i, color.red, color.green, color.blue);
        }
        return buffer;
    }

}
