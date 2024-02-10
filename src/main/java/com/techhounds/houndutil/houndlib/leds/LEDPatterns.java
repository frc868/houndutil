package com.techhounds.houndutil.houndlib.leds;

import java.util.function.Consumer;

import com.techhounds.houndutil.houndlib.IntegerContainer;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;

public class LEDPatterns {
    /**
     * Changes the contents of the AddressableLEDBuffer to the rainbow state.
     */

    public static Consumer<AddressableLEDBuffer> solid(Color color, BaseLEDSection section) {
        return (AddressableLEDBuffer buffer) -> {
            for (int i = section.getStart(); i <= section.getEnd(); i++) {
                buffer.setLED(i, color);
            }
        };
    }

    /**
     * 
     * @param section
     * @param brightness
     * @param speed      between 0 and 180
     * @return
     */
    public static Consumer<AddressableLEDBuffer> rainbow(int brightness, int speed, BaseLEDSection section) {
        IntegerContainer firstPixelHue = new IntegerContainer(0);

        return (AddressableLEDBuffer buffer) -> {
            for (int i = section.getStart(); i <= section.getEnd(); i++) {
                // Calculate the hue - hue is easier for rainbows because the color
                // shape is a circle so only one value needs to change
                final var hue = (firstPixelHue.value + (i * 180 / buffer.getLength())) % 180;
                buffer.setHSV(i, hue, 255, brightness);
            }
            // Increase to make the rainbow "move"
            firstPixelHue.value += speed;
            // Check bounds
            firstPixelHue.value %= 180;
        };
    }

    public static Consumer<AddressableLEDBuffer> flash(Color color, double onTime, BaseLEDSection section) {
        return (AddressableLEDBuffer buffer) -> {
            for (int i = section.getStart(); i <= section.getEnd(); i++) {
                if (Timer.getFPGATimestamp() % (onTime * 2) > onTime) {
                    buffer.setLED(i, color);
                } else {
                    buffer.setLED(i, Color.kBlack);
                }
            }
        };
    }
}
