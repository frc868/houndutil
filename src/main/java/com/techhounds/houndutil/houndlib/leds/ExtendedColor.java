package com.techhounds.houndutil.houndlib.leds;

import edu.wpi.first.wpilibj.util.Color;

public class ExtendedColor {
    public final Color color;

    public ExtendedColor(Color color) {
        this.color = color;
    }

    public final int hue() {
        return RGBtoHSV(color.red, color.green, color.blue)[0];
    }

    public final int saturation() {
        return RGBtoHSV(color.red, color.green, color.blue)[1];
    }

    public final int value() {
        return RGBtoHSV(color.red, color.green, color.blue)[2];
    }

    public final int red() {
        return (int) (color.red * 255);
    }

    public final int green() {
        return (int) (color.green * 255);
    }

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
        double cMax = Math.max(red, Math.max(green, blue));
        double cMin = Math.min(red, Math.min(green, blue));

        double delta = cMax - cMin;

        // Hue
        int hue;

        if (delta == 0) {
            hue = 0;
        } else if (cMax == red) {
            hue = (int) Math.round(60 * (((green - blue) / delta) % 6));
        } else if (cMax == green) {
            hue = (int) Math.round(60 * (((blue - red) / delta) + 2));
        } else {
            hue = (int) Math.round(60 * (((red - green) / delta) + 4));
        }

        // Saturation
        double saturation = (cMax == 0) ? 0 : delta / cMax;

        // Convert final values to correct range
        return new int[] {
                hue / 2,
                (int) Math.round(saturation * 255),
                (int) Math.round(cMax * 255) };
    }

}