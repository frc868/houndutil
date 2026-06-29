package com.techhounds.houndutil.houndlib.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FinishedSubsystemBase extends SubsystemBase{
    public static double[] createTuningConstants(double kA, double kD, double kG, double kI, double kP, double kS, double kV){
        return new double[]{kA, kD, kG, kI, kP, kS, kV};
    }
}
