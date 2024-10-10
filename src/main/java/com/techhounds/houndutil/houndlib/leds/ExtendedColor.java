package com.techhounds.houndutil.houndlib.leds;

import edu.wpi.first.wpilibj.util.Color;

/**
 * A wrapper class that provides conversions between RGB values and HSV values.
 */
public class ExtendedColor {
    /**
     * The base Color object.
     */
    public final Color color;

    /**
     * Creates a new ExtendedColor.
     * 
     * @param color the {@link Color} object to wrap
     */
    public ExtendedColor(Color color) {
        this.color = color;
    }

    /**
     * Gets the hue of the color.
     * 
     * @return the hue, from 0-180
     */
    public final int hue() {
        return RGBtoHSV(color.red, color.green, color.blue)[0];
    }

    /**
     * Gets the saturation of the color.
     * 
     * @return the saturation, from 0-255
     */
    public final int saturation() {
        return RGBtoHSV(color.red, color.green, color.blue)[1];
    }

    /**
     * Gets the value of the color.
     * 
     * @return the value, from 0-255
     */
    public final int value() {
        return RGBtoHSV(color.red, color.green, color.blue)[2];
    }

    /**
     * Gets the red value of the color.
     * 
     * @return the red value, from 0-255
     */
    public final int red() {
        return (int) (color.red * 255);
    }

    /**
     * Gets the green value of the color.
     * 
     * @return the green value, from 0-255
     */
    public final int green() {
        return (int) (color.green * 255);
    }

    /**
     * Gets the blue value of the color.
     * 
     * @return the blue value, from 0-255
     */
    public final int blue() {
        return (int) (color.blue * 255);
    }

    /**
     * Converts an RGB color to HSV.
     * 
     * @param r The red value of the color [0-1].
     * @param g The green value of the color [0-1].
     * @param b The blue value of the color [0-1].
     */
    private static int[] RGBtoHSV(double red, double green, double blue) {
        // wikipedia math
        double cMax = Math.max(red, Math.max(green, blue));
        double cMin = Math.min(red, Math.min(green, blue));

        double delta = cMax - cMin;

        int hue;
        if (delta == 0) {
            hue = 0;
        } else if (cMax == red) {
            hue = (int) Math.round(60 * (((green - blue) / delta + 6) % 6));
        } else if (cMax == green) {
            hue = (int) Math.round(60 * (((blue - red) / delta) + 2));
        } else { // cMax == blue
            hue = (int) Math.round(60 * (((red - green) / delta) + 4));
        }

        double saturation = (cMax == 0) ? 0 : delta / cMax;

        // Convert final values to correct range
        return new int[] {
                hue / 2,
                (int) Math.round(saturation * 255),
                (int) Math.round(cMax * 255) };
    }

}