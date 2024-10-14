package org.xero1425.subsystems.oi;

import edu.wpi.first.wpilibj.DriverStation;

public class OIInputsOutputsHID implements OIInputsOutputs {
    private int port_ ;

    public OIInputsOutputsHID(int port) {
        port_  = port ;
    }    

    public void updateInputs(OIInputs inputs) {
        for(int i = 1 ; i <= 32 ; i++) {
            inputs.buttons_[i - 1] = DriverStation.getStickButton(port_, i) ;
        }
    }

    public void setLED(int index, boolean on) {
    }    
}
