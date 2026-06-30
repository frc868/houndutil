package com.techhounds.houndutil.houndlib.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FinishedSubsystemBase extends SubsystemBase{
    public enum KrackenType{
        SIXTY(60),
        FORTY_FOUR(44);

        private KrackenType(int n){
            this.n = n;
        }

        private int n;

        public int getInt() {
            return n;
        }
    }
}