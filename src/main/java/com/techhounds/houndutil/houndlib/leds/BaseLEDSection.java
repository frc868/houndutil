package com.techhounds.houndutil.houndlib.leds;

public interface BaseLEDSection {
    public int start();

    public int end();

    public int length();

    public boolean inverted();
}
