package com.techhounds.houndutil.houndlib.subsystems.finished;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FinishedSubsystemBase extends SubsystemBase{
    public enum KrakenType{
        SIXTY(60),
        FORTY_FOUR(44);

        private KrakenType(int n){
            this.n = n;
        }

        private int n;

        public int getInt() {
            return n;
        }
    }
}
