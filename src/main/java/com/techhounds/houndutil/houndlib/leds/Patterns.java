package com.techhounds.houndutil.houndlib.leds;

import com.techhounds.houndutil.houndlib.ValueContainer;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;

public class Patterns {
    /**
     * Changes the contents of the AddressableLEDBuffer to the rainbow state.
     */

    public static Runnable rainbow(double firstIndex, double lastIndex, AddressableLEDBuffer buffer) {
        ValueContainer firstPixelHue = new ValueContainer(0);

        return () -> {
            for (int i = 0; i < buffer.getLength(); i++) {
                // Calculate the hue - hue is easier for rainbows because the color
                // shape is a circle so only one value needs to change
                final var hue = (firstPixelHue.value + (i * 180 / buffer.getLength())) % 180;
                // Set the value
                buffer.setHSV(i, hue, 255, 255); // max brightness until adjusted later

            }
            // Increase by 3 to make the rainbow "move"
            firstPixelHue.value += 3;
            // Check bounds
            firstPixelHue.value %= 180;
        };
    }

    public static Runnable solid(Color color, double firstIndex, double lastIndex, AddressableLEDBuffer buffer) {
        return () -> {
            for (int i = 0; i < buffer.getLength(); i++) {
                buffer.setLED(i, color);
            }
        };
    }

    public static Runnable flash(Color color, double onDuration, double firstIndex, double lastIndex,
            AddressableLEDBuffer buffer) {
        return () -> {

            for (int i = 0; i < buffer.getLength(); i++) {
                if (Timer.getFPGATimestamp() % (onDuration * 2) > onDuration) {
                    buffer.setLED(i, color);
                } else {
                    buffer.setLED(i, Color.kBlack);
                }
            }
        };
    }

    /**
     * Changes the contents of the AddressableLEDBuffer to the TechHOUNDS state.
     */
    public static void techHounds(AddressableLEDBuffer buffer) {
        ValueContainer timeStep = new ValueContainer(0);

        for (int i = 0; i < buffer.getLength(); i++) {
            if (((i + timeStep.value) / 8) % 2 == 0) { // shifts back and forth every 8 pixels
                buffer.setHSV(i, 15, 250, 255); // gold
            } else {
                buffer.setHSV(i, 109, 240, 255); // blue
            }
        }
        timeStep.value += 1;
    }
}
