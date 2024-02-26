package com.techhounds.houndutil.houndlib;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class ChassisAccelerations {
    public double axMetersPerSecondSquared;
    public double ayMetersPerSecondSquared;
    public double omegaRadiansPerSecondSquared;

    public ChassisAccelerations(double axMetersPerSecondSquared, double ayMetersPerSecondSquared,
            double omegaRadiansPerSecondSquared) {
        this.axMetersPerSecondSquared = axMetersPerSecondSquared;
        this.ayMetersPerSecondSquared = ayMetersPerSecondSquared;
        this.omegaRadiansPerSecondSquared = omegaRadiansPerSecondSquared;
    }

    public ChassisAccelerations(ChassisSpeeds speed, ChassisSpeeds previousSpeed, double dt) {
        this.axMetersPerSecondSquared = (speed.vxMetersPerSecond - previousSpeed.vxMetersPerSecond) / dt;
        this.ayMetersPerSecondSquared = (speed.vyMetersPerSecond - previousSpeed.vyMetersPerSecond) / dt;
        this.omegaRadiansPerSecondSquared = (speed.omegaRadiansPerSecond - previousSpeed.omegaRadiansPerSecond) / dt;
    }
}