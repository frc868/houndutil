package com.techhounds.houndutil.houndlib.leds;

import java.util.List;
import java.util.Random;
import java.util.function.Consumer;

import com.techhounds.houndutil.houndlib.DoubleContainer;
import com.techhounds.houndutil.houndlib.IntegerContainer;
import com.techhounds.houndutil.houndlib.Utils;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;

/**
 * A class providing static consumers of {@link AddressableLEDBuffer}s to create
 * specific LED patterns.
 */
public class LEDPatterns {
    /**
     * Creates a solid LED pattern.
     * 
     * @param color   the color to set the LED section to
     * @param section the LED section to use
     * @return the generated LED buffer consumer
     */
    public static Consumer<AddressableLEDBuffer> solid(Color color, BaseLEDSection section) {
        return (AddressableLEDBuffer buffer) -> {
            for (int i = section.start(); i <= section.end(); i++) {
                buffer.setLED(i, color);
            }
        };
    }

    /**
     * Creates a moving rainbow pattern.
     * 
     * @param brightness the brightness [0-255] of the colors used for the rainbow
     * @param speed      the amount to move the first hue value by each timestep
     *                   (recommended 3)
     * @param section    the LED section to use
     * @return the generated LED buffer consumer
     */
    public static Consumer<AddressableLEDBuffer> rainbow(int brightness, int speed, BaseLEDSection section) {
        IntegerContainer firstPixelHue = new IntegerContainer(0);

        return (AddressableLEDBuffer buffer) -> {
            if (section.inverted()) {
                for (int i = 0; i < section.length(); i++) {
                    // Calculate the hue - hue is easier for rainbows because the color
                    // shape is a circle so only one value needs to change
                    final var hue = (firstPixelHue.value + (i * 180 / section.length())) % 180;
                    buffer.setHSV(section.end() - i, hue, 255, brightness);
                }
            } else {
                for (int i = 0; i <= section.length(); i++) {
                    // Calculate the hue - hue is easier for rainbows because the color
                    // shape is a circle so only one value needs to change
                    final var hue = (firstPixelHue.value + (i * 180 / section.length())) % 180;
                    buffer.setHSV(i + section.start(), hue, 255, brightness);
                }
            }
            // Increase to make the rainbow "move"
            firstPixelHue.value += speed;
            // Check bounds
            firstPixelHue.value %= 180;
        };
    }

    /**
     * Creates a pattern that flashes a specific color.
     * 
     * @param brightness the brightness [0-255] of the colors used for the rainbow
     * @param speed      the amount to move the first hue value by each timestep
     *                   (recommended 3)
     * @param section    the LED section to use
     * @return the generated LED buffer consumer
     */
    public static Consumer<AddressableLEDBuffer> flash(Color color, double onTime, BaseLEDSection section) {
        return (AddressableLEDBuffer buffer) -> {
            for (int i = section.start(); i <= section.end(); i++) {
                if (Timer.getFPGATimestamp() % (onTime * 2) > onTime) {
                    buffer.setLED(i, color);
                } else {
                    buffer.setLED(i, Color.kBlack);
                }
            }
        };
    }

    /**
     * Creates a pattern that triggers a bolt with a decreasing brightness tail
     * across
     * the LED section, similar to something being chased.
     * 
     * @param color    the color to set the bolt to
     * @param length   the length of the bolt
     * @param time     the amount of time to take to finish the movement of the bolt
     *                 across the LED strip, and optionally the time after which to
     *                 send a new bolt
     * @param multiple whether to continuously send bolts
     * @param section  the LED section to use
     * @return the generated LED buffer consumer
     */
    public static Consumer<AddressableLEDBuffer> chase(Color color, double length, double time, boolean multiple,
            BaseLEDSection section) {
        DoubleContainer previousTime = new DoubleContainer(0.0);
        // store fractional changes in the movement
        DoubleContainer movementBuffer = new DoubleContainer(0.0);
        IntegerContainer startPoint = new IntegerContainer(0);
        ExtendedColor extColor = new ExtendedColor(color);

        return (AddressableLEDBuffer buffer) -> {
            double currentTime = Timer.getFPGATimestamp();
            double elapsedTime = currentTime - previousTime.value;
            previousTime.value = currentTime;
            // reset everything if it's been a while since we've run this continuously
            if (elapsedTime > 0.1) {
                elapsedTime = 0.0;
                startPoint.value = 0;
                movementBuffer.value = 0.0;
            }

            double stepPerLed = 255.0 / length;

            movementBuffer.value += elapsedTime * ((section.length() + length) / time);
            if (movementBuffer.value > 1) {
                startPoint.value += (int) movementBuffer.value;
                if (multiple)
                    startPoint.value %= (section.length() + length);
                movementBuffer.value %= 1;
            }

            // start at the start point and work backwards
            // go till the end of the strip and cut off the end when i = 0
            for (int i = startPoint.value; i > startPoint.value - length && i >= 0; i--) {
                // if we're beyond the end of the strip, animate the final tail
                if (i < section.length()) {
                    int value = (int) (255 - (stepPerLed * (startPoint.value - i)));
                    buffer.setHSV(i + section.start(), extColor.hue(), extColor.saturation(), value);
                }
            }
        };
    }

    /**
     * Creates a pattern that triggers a bolt with a decreasing brightness tail
     * across the LED section, similar to something being chased. Interpolates
     * between two colors.
     * 
     * @param primaryColor   the primary (starting) color to set the bolt to
     * @param secondaryColor the secondary (ending) color to set the bolt to
     * @param length         the length of the bolt
     * @param time           the amount of time to take to finish the movement of
     *                       the bolt
     *                       across the LED strip, and optionally the time after
     *                       which to
     *                       send a new bolt
     * @param minBrightness  the brightness [0-255] at the end of the tail, used to
     *                       ensure that the secondary color is visible
     * @param multiple       whether to continuously send bolts
     * @param section        the LED section to use
     * @return the generated LED buffer consumer
     */
    public static Consumer<AddressableLEDBuffer> chase(Color primaryColor, Color secondaryColor, double length,
            double time, int minBrightness, boolean multiple,
            BaseLEDSection section) {
        DoubleContainer previousTime = new DoubleContainer(0.0);
        // store fractional changes in the movement
        DoubleContainer movementBuffer = new DoubleContainer(0.0);
        IntegerContainer startPoint = new IntegerContainer(0);
        ExtendedColor extPrimaryColor = new ExtendedColor(primaryColor);
        ExtendedColor extSecondaryColor = new ExtendedColor(secondaryColor);

        return (AddressableLEDBuffer buffer) -> {
            double currentTime = Timer.getFPGATimestamp();
            double elapsedTime = currentTime - previousTime.value;
            previousTime.value = currentTime;
            // reset everything if it's been a while since we've run this continuously
            if (elapsedTime > 0.1) {
                elapsedTime = 0.0;
                startPoint.value = 0;
                movementBuffer.value = 0.0;
            }

            movementBuffer.value += elapsedTime * ((section.length() + length) / time);
            if (movementBuffer.value > 1) {
                startPoint.value += (int) movementBuffer.value;
                if (multiple)
                    startPoint.value %= (section.length() + length);
                movementBuffer.value %= 1;
            }

            // start at the start point and work backwards
            // go till the end of the strip and cut off the end when i = 0
            for (int i = startPoint.value; i > startPoint.value - length && i >= 0; i--) {
                // if we're beyond the end of the strip, animate the final tail
                if (i < section.length()) {
                    double interpolationValue = (startPoint.value - i) / length;
                    int value = (int) ((255 - minBrightness) * (1 - interpolationValue)) + minBrightness;
                    buffer.setHSV(i + section.start(),
                            Utils.interpolate(extPrimaryColor.hue(), extSecondaryColor.hue(), interpolationValue),
                            Utils.interpolate(extPrimaryColor.saturation(), extSecondaryColor.saturation(),
                                    interpolationValue),
                            value);
                }
            }
        };
    }

    /**
     * Creates a pattern that simulates a wave, with transition between a low and
     * high brightness version of a color that moves through the strip using a sine
     * function for smoothness.
     * 
     * @param color         the color to set the wave to
     * @param length        the length of one period of the wave
     * @param waveSpeed     the number of radians to shift the wave per second
     *                      (recommended 1-3)
     * @param minBrightness the brightness at the lowest point of the wave
     * @param maxBrightness the brightness at the highest point of the wave
     * @param section       the LED section to use
     * @return the generated LED buffer consumer
     */
    public static Consumer<AddressableLEDBuffer> wave(Color color, int length, double waveSpeed, int minBrightness,
            int maxBrightness,
            BaseLEDSection section) {
        ExtendedColor extColor = new ExtendedColor(color);

        return (AddressableLEDBuffer buffer) -> {
            double currentTime = Timer.getFPGATimestamp();
            // The wave's phase shift per iteration to create movement
            double phaseShift = currentTime * waveSpeed;

            for (int i = 0; i < section.length(); i++) {
                // Calculate wave position for each LED
                double wavePosition = (2 * Math.PI / length) * i - phaseShift;
                // Calculate brightness value using sine function
                // normalize sine output
                int value = (int) ((Math.sin(wavePosition) + 1) / 2 * (maxBrightness - minBrightness)
                        + minBrightness);
                if (section.inverted()) {
                    buffer.setHSV(section.end() - i, extColor.hue(), extColor.saturation(), value);
                } else {
                    buffer.setHSV(i + section.start(), extColor.hue(), extColor.saturation(), value);
                }
            }
        };
    }

    /**
     * Creates a pattern that simulates a wave that continously changes in color
     * over a rainbow.
     * 
     * @param rainbowSpeed  the amount to move the first hue value by each timestep
     *                      (recommended 3)
     * @param length        the length of one period of the wave
     * @param waveSpeed     the number of radians to shift the wave per second
     *                      (recommended 1-3)
     * @param minBrightness the brightness at the lowest point of the wave
     * @param maxBrightness the brightness at the highest point of the wave
     * @param section       the LED section to use
     * @return the generated LED buffer consumer
     */
    public static Consumer<AddressableLEDBuffer> waveRainbow(double rainbowSpeed, int length, double waveSpeed,
            int minBrightness,
            int maxBrightness,
            BaseLEDSection section) {
        IntegerContainer hue = new IntegerContainer(0);

        return (AddressableLEDBuffer buffer) -> {
            ExtendedColor extColor = new ExtendedColor(Color.fromHSV(hue.value, 255, 255));
            double currentTime = Timer.getFPGATimestamp();
            // The wave's phase shift per iteration to create movement
            double phaseShift = currentTime * waveSpeed;

            for (int i = 0; i < section.length(); i++) {
                // Calculate wave position for each LED
                double wavePosition = (2 * Math.PI / length) * i - phaseShift;
                // Calculate brightness value using sine function
                // normalize sine output
                int value = (int) ((Math.sin(wavePosition) + 1) / 2 * (maxBrightness - minBrightness)
                        + minBrightness);
                if (section.inverted()) {
                    buffer.setHSV(section.end() - i, extColor.hue(), extColor.saturation(), value);
                } else {
                    buffer.setHSV(i + section.start(), extColor.hue(), extColor.saturation(), value);
                }
            }
            hue.value += rainbowSpeed;
            hue.value %= 180;
        };
    }

    /**
     * Creates a pattern that moves a color between high and low brightness via a
     * sine wave, which looks like "breathing".
     * 
     * @param color         the color to breathe with
     * @param onTime        the amount of time, in seconds, that the strip should be
     *                      on for
     * @param minBrightness the brightness at the lowest point of the breath (make
     *                      this 0 to turn off the strip)
     * @param maxBrightness the brightness at the highest point of the breath
     * @param section       the LED section to use
     * @return the generated LED buffer consumer
     */
    public static Consumer<AddressableLEDBuffer> breathe(Color color, double onTime,
            int minBrightness, int maxBrightness, BaseLEDSection section) {
        ExtendedColor extColor = new ExtendedColor(color);

        return (AddressableLEDBuffer buffer) -> {
            double currentTime = Timer.getFPGATimestamp();
            // Calculate the sine wave phase based on the current time and onTime as the
            // period for one complete cycle
            double phase = (2 * Math.PI / onTime) * (currentTime % onTime);
            // Normalize sine wave output to oscillate between minBrightness and
            // maxBrightness
            int value = (int) (minBrightness + ((Math.sin(phase) + 1) / 2 * (maxBrightness - minBrightness)));

            for (int i = 0; i < section.length(); i++) {
                buffer.setHSV(i + section.start(), extColor.hue(), extColor.saturation(), value);
            }
        };
    }

    /**
     * Creates a pattern that creates a flickering effect similar to a fire, with
     * the base colors of a fire (red, yellow, and white). Algorithm derivative of
     * Fire2012.
     * 
     * @param sparking percentage likelihood of a new spark being lit.
     *                 higher chance = more roaring fire, lower chance = more
     *                 flickery fire. [0-1]
     * @param cooling  how much the air cools as it rises.
     *                 less cooling = taller flames, more cooling = shorter flames.
     *                 [0-1]
     * @param section  the LED section to use
     * @return the generated LED buffer consumer
     */
    public static Consumer<AddressableLEDBuffer> fire2012(double sparking, double cooling,
            BaseLEDSection section) {
        Random random = new Random();
        int[] heat = new int[section.length()];

        return (AddressableLEDBuffer buffer) -> {
            // Cool down every cell a little
            for (int i = 0; i < section.length(); i++) {
                heat[i] = Math.max(0, heat[i] - random.nextInt((int) (cooling * 10 * 255 / section.length()) + 2));
            }

            // Heat from each cell drifts 'up' and diffuses a little
            for (int i = section.length() - 1; i > 2; i--) {
                heat[i] = (heat[i - 1] + heat[i - 2] + heat[i - 2]) / 3;
            }

            // Randomly ignite new 'sparks' of heat near the bottom
            if (random.nextDouble() < sparking) {
                int y = random.nextInt(3);
                heat[y] = Math.min(heat[y] + random.nextInt(160, 255), 255);
            }

            // // Convert heat to LED colors
            for (int i = 0, j = section.start(); i < section.length() && j < section.end(); i++, j++) {
                Color color = interpolateHeat(heat[i]);
                buffer.setLED(j, color);
            }
        };
    }

    /**
     * Creates a pattern that creates a flickering effect similar to a fire, using a
     * specfic palette of colors. Algorithm derivative of Fire2012.
     * 
     * @param sparking percentage likelihood of a new spark being lit.
     *                 higher chance = more roaring fire, lower chance = more
     *                 flickery fire. [0-1]
     * @param cooling  how much the air cools as it rises.
     *                 less cooling = taller flames, more cooling = shorter flames.
     *                 [0-1]
     * @param colors   the list of colors to interpolate between. the first color
     *                 indicates the lowest temperature (typically black), and the
     *                 last color indicates the highest temperature. a palette
     *                 resembling default Fire2012 would be
     *                 {@code List.of(Color.kBlack, Color.kRed, Color.kYellow, Color.kWhite)}.
     * @param section  the LED section to use
     * @return the generated LED buffer consumer
     */
    public static Consumer<AddressableLEDBuffer> fire2012Palette(double sparking, double cooling, List<Color> colors,
            BaseLEDSection section) {
        Random random = new Random();
        int[] heat = new int[section.length()];

        return (AddressableLEDBuffer buffer) -> {
            // Cool down every cell a little
            for (int i = 0; i < section.length(); i++) {
                heat[i] = Math.max(0, heat[i] - random.nextInt((int) (cooling * 10 * 255 / section.length()) + 2));
            }

            // Heat from each cell drifts 'up' and diffuses a little
            for (int i = section.length() - 1; i > 2; i--) {
                heat[i] = (heat[i - 1] + heat[i - 2] + heat[i - 2]) / 3;
            }

            // Randomly ignite new 'sparks' of heat near the bottom
            if (random.nextDouble() < sparking) {
                int y = random.nextInt(3);
                heat[y] = Math.min(heat[y] + random.nextInt(160, 255), 255);
            }

            // // Convert heat to LED colors

            for (int i = 0, j = section.start(); i < section.length() && j < section.end(); i++, j++) {
                Color color = interpolateHeat(heat[i], colors);
                if (section.inverted())
                    buffer.setLED(section.end() - i, color);
                else
                    buffer.setLED(j, color);
            }
        };
    }

    /**
     * Creates a pattern that creates a flickering effect similar to a fire, with
     * a moving rainbow as a base palette. Algorithm derivative of Fire2012.
     * 
     * @param sparking     percentage likelihood of a new spark being lit.
     *                     higher chance = more roaring fire, lower chance = more
     *                     flickery fire. [0-1]
     * @param cooling      how much the air cools as it rises.
     *                     less cooling = taller flames, more cooling = shorter
     *                     flames. [0-1]
     * @param rainbowSpeed the amount to move the first hue value by each timestep
     *                     (recommended 3)
     * @param section      the LED section to use
     * @return the generated LED buffer consumer
     */
    public static Consumer<AddressableLEDBuffer> fire2012Rainbow(double sparking, double cooling, double rainbowSpeed,
            BaseLEDSection section) {
        Random random = new Random();
        int[] heat = new int[section.length()];
        IntegerContainer hue = new IntegerContainer(0);

        return (AddressableLEDBuffer buffer) -> {
            List<Color> colors = List.of(Color.kBlack, Color.fromHSV(hue.value, 255, 192),
                    Color.fromHSV(hue.value, 128, 255), Color.kWhite);
            // Cool down every cell a little
            for (int i = 0; i < section.length(); i++) {
                heat[i] = Math.max(0, heat[i] - random.nextInt((int) (cooling * 10 * 255 / section.length()) + 2));
            }

            // Heat from each cell drifts 'up' and diffuses a little
            for (int i = section.length() - 1; i > 2; i--) {
                heat[i] = (heat[i - 1] + heat[i - 2] + heat[i - 2]) / 3;
            }

            // Randomly ignite new 'sparks' of heat near the bottom
            if (random.nextDouble() < sparking) {
                int y = random.nextInt(3);
                heat[y] = Math.min(heat[y] + random.nextInt(160, 255), 255);
            }

            // // Convert heat to LED colors

            for (int i = 0, j = section.start(); i < section.length() && j < section.end(); i++, j++) {
                Color color = interpolateHeat(heat[i], colors);
                if (section.inverted())
                    buffer.setLED(section.end() - i, color);
                else
                    buffer.setLED(j, color);
            }
            hue.value += rainbowSpeed;
            hue.value %= 180;
        };
    }

    /**
     * Interpolates a standard fire color from a heat value from 0-255.
     * 
     * Algorithm derived from Fire2012.
     * 
     * @param heat the "heat" value as a temperature [0-255]
     * @return the color of the fire corresponding to that heat
     */
    private static Color interpolateHeat(int heat) {
        int t192 = (int) (heat / 255.0 * 192);

        int heatramp = (t192 % 64) * 4;

        int red;
        int green;
        int blue;
        if (t192 >= 128) {
            red = 255;
            green = 255;
            blue = heatramp;
        } else if (t192 >= 64) {
            red = 255; // full red
            green = heatramp; // ramp up green
            blue = 0; // no blue
        } else {
            red = heatramp; // full red
            green = 0; // ramp up green
            blue = 0; // no blue
        }

        return new Color(red, green, blue);
    }

    /**
     * Interpolates a fire color based on a palette of colors, from a heat value
     * from 0-255.
     * 
     * @param heat   the "heat" value as a temperature [0-255]
     * @param colors the colors to use in the interpolation. the first color is the
     *               lowest heat, and the last color is the highest heat. a good
     *               approximation to the original Fire2012 animation is [Black,
     *               Red, Yellow, White].
     * @return the color of the fire corresponding to that heat
     */
    private static Color interpolateHeat(int heat, List<Color> colors) {
        double scaled01 = heat / 255.0;

        double scaledValue = scaled01 * (colors.size() - 1);
        int index = (int) scaledValue;
        double t = scaledValue - index; // Fractional part of the scaled value

        // Perform linear interpolation
        Color color1 = colors.get(index);
        Color color2 = colors.get(Math.min(index + 1, colors.size() - 1));

        double red = Utils.interpolate(color1.red, color2.red, t);
        double green = Utils.interpolate(color1.green, color2.green, t);
        double blue = Utils.interpolate(color1.blue, color2.blue, t);

        return new Color(red, green, blue);
    }

}
