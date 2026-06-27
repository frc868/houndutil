package com.techhounds.houndutil.houndlib.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FinishedSuperstructure extends SubsystemBase {
    public final boolean isDrivetrain;
    public final boolean isFeeder;
    public final boolean isHopper;
    public final boolean isIntake;
    public final boolean isShooter;
    public final boolean isShooterHood;
    public final boolean isVision;

    public FinishedSuperstructure (boolean isDrivetrain, boolean isFeeder, boolean isHopper, boolean isIntake, boolean isShooter, boolean isShooterHood, boolean isVision) {
        this.isDrivetrain = isDrivetrain;
        this.isFeeder = isFeeder;
        this.isHopper = isHopper;
        this.isIntake = isIntake;
        this.isShooter = isShooter;
        this.isShooterHood = isShooterHood;
        this.isVision = isVision;
    }

    public void createDrivetrain (boolean isDrivetrain) {
        if (isDrivetrain) {
            
        }
    }
}
