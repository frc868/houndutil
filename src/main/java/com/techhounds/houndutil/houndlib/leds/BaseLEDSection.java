package com.techhounds.houndutil.houndlib.leds;

/**
 * A base for sections of an LED strip that defines start and end points, a
 * length, and whether the strip is inverted.
 */
public interface BaseLEDSection {
    /**
     * Gets the start point of the strip section as an index of the
     * {@link AddressableLEDBuffer}.
     * 
     * @return the start point
     */
    public int start();

    /**
     * Gets the end point of the strip section as an index of the
     * {@link AddressableLEDBuffer}.
     * 
     * @return the end point
     */
    public int end();

    /**
     * Gets the number of LEDs in this section of the strip.
     * 
     * @return the number of LEDs
     */
    public int length();

    /**
     * Gets whether this section of the strip should have its effects inverted
     * (starting effects at {@code end()} instead of {@code start()}).
     * 
     * @return whether effects should be inverted
     */
    public boolean inverted();
}
