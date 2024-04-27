package com.techhounds.houndutil.houndlib.leds;

import java.util.List;
import java.util.Random;
import java.util.function.Consumer;

import com.techhounds.houndutil.houndlib.DoubleContainer;
import com.techhounds.houndutil.houndlib.IntegerContainer;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;

public class LEDPatterns {
    private static int interpolate(int start, int end, double interpolationValue) {
        return (int) (start + (end - start) * interpolationValue);
    }

    /**
     * Changes the contents of the AddressableLEDBuffer to the rainbow state.
     */

    public static Consumer<AddressableLEDBuffer> solid(Color color, BaseLEDSection section) {
        return (AddressableLEDBuffer buffer) -> {
            for (int i = section.start(); i <= section.end(); i++) {
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
     * 
     * @param color
     * @param length
     * @param speed   LEDs per second
     * @param section
     * @return
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
     * 
     * @param color
     * @param length
     * @param speed   LEDs per second
     * @param section
     * @return
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
                            interpolate(extPrimaryColor.hue(), extSecondaryColor.hue(), interpolationValue),
                            interpolate(extPrimaryColor.saturation(), extSecondaryColor.saturation(),
                                    interpolationValue),
                            value);
                }
            }
        };
    }

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
                buffer.setHSV(i + section.start(), extColor.hue(), extColor.saturation(), value);
            }
        };
    }

    public static Consumer<AddressableLEDBuffer> breathe(Color color, double onTime,
            int minBrightness, int maxBrightness, BaseLEDSection section) {
        ExtendedColor extColor = new ExtendedColor(color);
        // Extract the color's brightness as the maximum brightness value

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

    // public static Consumer<AddressableLEDBuffer> fire(double sparking, double
    // cooling, double speed,
    // BaseLEDSection section, Color primaryColor, Color secondaryColor, Color
    // tertiaryColor, int flameHeight) {
    // Random random = new Random();
    // int[] heat = new int[section.end() - section.start() + 1];
    // int maxHeatIndex = Math.min(flameHeight, heat.length) - 1; // Ensure
    // flameHeight does not exceed heat array
    // // bounds

    // return (AddressableLEDBuffer buffer) -> {
    // // Cool down every cell a little
    // for (int i = section.start(); i <= section.end(); i++) {
    // heat[i - section.start()] = Math.max(0,
    // heat[i - section.start()] - random.nextInt((int) (cooling * 100 / speed) +
    // 2));
    // }

    // // Heat from each cell drifts 'up' and diffuses a little
    // for (int k = section.end(); k >= section.start() + 2; k--) {
    // heat[k - section.start()] = (heat[k - section.start() - 1] + heat[k -
    // section.start() - 2]
    // + heat[k - section.start() - 2]) / 3;
    // }

    // // Randomly ignite new 'sparks' of heat near the bottom
    // if (random.nextDouble() < sparking) {
    // int y = random.nextInt(Math.min(7, maxHeatIndex + 1));
    // heat[y] = Math.min(heat[y] + random.nextInt((int) (95 * speed)) + 160, 255);
    // }

    // System.out.println(Arrays.toString(heat));
    // // Convert heat to LED colors
    // for (int j = section.start(); j <= section.end(); j++) {
    // Color color = interpolateColor(primaryColor, secondaryColor, tertiaryColor,
    // heat[j - section.start()]);
    // buffer.setLED(j, color);
    // }
    // };
    // }

    /**
     * Algorithm derivative of Fire2012.
     * 
     * @param sparking       likelihood of a new spark being lit. higher chance =
     *                       more roaring fire, lower chance = more flickery fire.
     *                       [0-1]
     * @param cooling        how much the air cools as it rises. less cooling =
     *                       taller flames, more cooling = shorter flames. [0-1]
     * @param speed
     * @param section
     * @param primaryColor
     * @param secondaryColor
     * @param tertiaryColor
     * @param flameHeight
     * @return
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
                Color color = interpolateColor(heat[i]);
                buffer.setLED(j, color);
            }
        };
    }

    private static Color interpolateColor(int heat) {
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
                Color color = interpolateColor(heat[i], colors);
                if (section.inverted())
                    buffer.setLED(section.end() - i, color);
                else
                    buffer.setLED(j, color);
            }
        };
    }

    private static Color interpolateColor(int heat, List<Color> colors) {
        double scaled01 = heat / 255.0;

        double scaledValue = scaled01 * (colors.size() - 1);
        int index = (int) scaledValue;
        double t = scaledValue - index; // Fractional part of the scaled value

        // Perform linear interpolation
        Color color1 = colors.get(index);
        Color color2 = colors.get(Math.min(index + 1, colors.size() - 1));

        double red = lerp(color1.red, color2.red, t);
        double green = lerp(color1.green, color2.green, t);
        double blue = lerp(color1.blue, color2.blue, t);

        return new Color(red, green, blue);
    }

    private static double lerp(double start, double end, double t) {
        return start + (end - start) * t;
    }

}
