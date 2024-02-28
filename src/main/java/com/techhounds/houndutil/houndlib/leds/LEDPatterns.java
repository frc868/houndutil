package com.techhounds.houndutil.houndlib.leds;

import java.util.Arrays;
import java.util.Random;
import java.util.function.Consumer;

import com.techhounds.houndutil.houndlib.DoubleContainer;
import com.techhounds.houndutil.houndlib.IntegerContainer;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

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
            for (int i = section.start(); i <= section.end(); i++) {
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

    public static Consumer<AddressableLEDBuffer> fire(double sparking, double cooling, double speed,
            BaseLEDSection section, Color primaryColor, Color secondaryColor, Color tertiaryColor, int flameHeight) {
        Random random = new Random();
        int[] heat = new int[section.end() - section.start() + 1];
        int maxHeatIndex = Math.min(flameHeight, heat.length) - 1; // Ensure flameHeight does not exceed heat array
                                                                   // bounds

        return (AddressableLEDBuffer buffer) -> {
            // Cool down every cell a little
            for (int i = section.start(); i <= section.end(); i++) {
                heat[i - section.start()] = Math.max(0,
                        heat[i - section.start()] - random.nextInt((int) (cooling * 100 / speed) + 2));
            }

            // Heat from each cell drifts 'up' and diffuses a little
            for (int k = section.end(); k >= section.start() + 2; k--) {
                heat[k - section.start()] = (heat[k - section.start() - 1] + heat[k - section.start() - 2]
                        + heat[k - section.start() - 2]) / 3;
            }

            // Randomly ignite new 'sparks' of heat near the bottom
            if (random.nextDouble() < sparking) {
                int y = random.nextInt(Math.min(7, maxHeatIndex + 1));
                heat[y] = Math.min(heat[y] + random.nextInt((int) (95 * speed)) + 160, 255);
            }

            System.out.println(Arrays.toString(heat));
            // Convert heat to LED colors
            for (int j = section.start(); j <= section.end(); j++) {
                Color color = interpolateColor(primaryColor, secondaryColor, tertiaryColor, heat[j - section.start()]);
                buffer.setLED(j, color);
            }
        };
    }

    private static Color interpolateColor(Color primary, Color secondary, Color tertiary, int heat) {
        int maxHeat = 255;
        double heatRatio = heat / (double) maxHeat;
        double fadeToBlack = heatRatio;

        // Determine which two colors to blend between and the blend ratio
        Color startColor, endColor;
        double blendRatio;
        if (heat <= maxHeat / 3) {
            // Blend between primary and secondary
            startColor = primary;
            endColor = secondary;
            blendRatio = heatRatio * 3; // Scale ratio to 0-1 range within the first third
        } else if (heat <= 2 * maxHeat / 3) {
            // Blend between secondary and tertiary
            startColor = secondary;
            endColor = tertiary;
            blendRatio = (heatRatio - 1.0 / 3) * 3; // Scale ratio to 0-1 range within the second third
        } else {
            // Blend between tertiary and a bit of primary to smooth out the transition back
            // to start, if looping
            startColor = tertiary;
            endColor = primary;
            blendRatio = (heatRatio - 2.0 / 3) * 3; // Scale ratio to 0-1 range within the last third
        }

        // Calculate blended color
        int r = (int) ((startColor.red * 255 * (1 - blendRatio) + endColor.red * 255 * blendRatio) * fadeToBlack);
        int g = (int) ((startColor.green * 255 * (1 - blendRatio) + endColor.green * 255 * blendRatio) * fadeToBlack);
        int b = (int) ((startColor.blue * 255 * (1 - blendRatio) + endColor.blue * 255 * blendRatio) * fadeToBlack);

        // Clamp values to ensure they are within 0-255 range
        r = Math.min(Math.max(r, 0), 255);
        g = Math.min(Math.max(g, 0), 255);
        b = Math.min(Math.max(b, 0), 255);

        return new Color(r, g, b);
    }

}
