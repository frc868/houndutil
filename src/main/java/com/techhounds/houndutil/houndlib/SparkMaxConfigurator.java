package com.techhounds.houndutil.houndlib;

import com.revrobotics.CANSparkMax;

public class SparkMaxConfigurator {
    private final CANSparkMax sparkMax;

    public SparkMaxConfigurator(CANSparkMax sparkMax) {
        this.sparkMax = sparkMax;
        this.sparkMax.restoreFactoryDefaults();
    }

    public SparkMaxConfigurator withInverted(boolean isInverted) {
        sparkMax.setInverted(isInverted);
        return this;
    }

    public SparkMaxConfigurator withIdleMode(CANSparkMax.IdleMode idleMode) {
        sparkMax.setIdleMode(idleMode);
        return this;
    }

    public SparkMaxConfigurator withCurrentLimit(int limit) {
        sparkMax.setSmartCurrentLimit(limit);
        return this;
    }

    public SparkMaxConfigurator withCurrentLimit(int stallLimit, int freeLimit) {
        sparkMax.setSmartCurrentLimit(stallLimit, freeLimit);
        return this;
    }

    public SparkMaxConfigurator withSecondaryCurrentLimit(int limit) {
        sparkMax.setSecondaryCurrentLimit(limit);
        return this;
    }

    public SparkMaxConfigurator withPositionConversionFactor(double factor, boolean setVelocity) {
        sparkMax.getEncoder().setPositionConversionFactor(factor);
        if (setVelocity) {
            sparkMax.getEncoder().setVelocityConversionFactor(factor / 60.0);
        }
        return this;
    }

    public SparkMaxConfigurator withOpenLoopRampRate(double rate) {
        sparkMax.setOpenLoopRampRate(rate);
        return this;
    }

    public void burnFlash() {
        sparkMax.burnFlash();
    }
}
