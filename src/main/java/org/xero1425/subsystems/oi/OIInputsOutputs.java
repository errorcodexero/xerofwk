package org.xero1425.subsystems.oi;

import org.littletonrobotics.junction.AutoLog;

public interface OIInputsOutputs {
    @AutoLog
    public static class OIInputs {
        public boolean buttons_[] = new boolean[32] ;
    }

    public default void updateInputs(OIInputs inputs) {
    }

    public default void setLED(int index, boolean on) {
    }
}
