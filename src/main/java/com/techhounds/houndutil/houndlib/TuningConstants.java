package com.techhounds.houndutil.houndlib;

public class TuningConstants {
    private double kP;
    private double kI;
    private double kD;
    private double kS;
    private double kV;
    private double kA;
    private double kG;

    public double getkP() {
        return this.kP;
    }
    public double getkI() {
        return this.kI;
    }
    public double getkD() {
        return this.kD;
    }
    public double getkS() {
        return this.kS;
    }
    public double getkV() {
        return this.kV;
    }
    public double getkA() {
        return this.kA;
    }
    public double getkG() {
        return this.kG;
    }

    public TuningConstants(double kP, double kI, double kD, double kS, double kV, double kA, double kG){
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kS = kS;
        this.kV = kV;
        this.kA = kA;
        this.kG = kG;
    }

    public TuningConstants(double kP, double kI, double kD){
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

}
