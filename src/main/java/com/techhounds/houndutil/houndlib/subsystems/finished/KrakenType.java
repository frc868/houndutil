package com.techhounds.houndutil.houndlib.subsystems.finished;

public enum KrakenType {
    SIXTY(60),
    FORTY_FOUR(44);

    private KrakenType(int n) {
        this.n = n;
    }

    private int n;

    public int getInt() {
        return n;
    }
}