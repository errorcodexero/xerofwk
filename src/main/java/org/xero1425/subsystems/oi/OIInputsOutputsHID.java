package org.xero1425.subsystems.oi;

import org.xero1425.subsystems.oi.OISubsystem.LEDState;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;

public class OIInputsOutputsHID implements OIInputsOutputs {
    private static final int kFastLoopCount = 10 ;
    private static final int kSlowLoopCount = 25 ;

    private int port_ ;
    private boolean led_onoff_[];
    private OISubsystem.LEDState led_state_[] ;

    private boolean fast_on_off_ ;
    private int fast_on_off_loops_ ;
    private boolean slow_on_off_ ;
    private int slow_on_off_loops_ ;

    private int max_leds_ ;
    private int max_buttons_ ;

    private GenericHID hid_ ;

    public OIInputsOutputsHID(GenericHID hid, int maxbuttons, int maxleds) {
        hid_ = hid ;
        port_ = hid.getPort() ;

        max_buttons_ = maxbuttons ;
        max_leds_ = maxleds ;

        led_onoff_ = new boolean[maxleds] ;
        led_state_ = new OISubsystem.LEDState[maxleds] ;
        for(int i = 0 ; i < max_leds_ ; i++) {
            led_state_[i] = LEDState.Off ;
        }

        fast_on_off_ = false ;
        fast_on_off_loops_ = 0 ;
        slow_on_off_ = false ;
        slow_on_off_loops_ = 0 ;
    }

    public void updateInputs(OIInputs inputs) {
        for(int i = 1 ; i <= max_buttons_ ; i++) {
            inputs.buttons_[i - 1] = DriverStation.getStickButton(port_, i) ;
        }

        for(int i = 1 ; i <= max_leds_ ; i++) {
            inputs.leds_[i-1] = led_onoff_[i - 1] ;
        }

        inputs.ledstates_ = getLEDStatesString() ;
    }

    public void setLEDState(int index, OISubsystem.LEDState st) {
        led_state_[index - 1] = st ;
    }

    private String getLEDStatesString() {
        String ret = "" ;

        for(int i = 1 ; i <= max_leds_ ; i++) {
            if (led_state_[i - 1] != LEDState.Off) {
                if (ret.length() > 0) {
                    ret += "," ;
                }

                ret += Integer.toString(i) ;
                ret += ":" + led_state_[i - 1].toString() ;
            } 
        }

        return ret ;
    }    

    public void updateLEDs() {
        fast_on_off_loops_++ ;
        if (fast_on_off_loops_ == kFastLoopCount) {
            fast_on_off_ = !fast_on_off_ ;
            fast_on_off_loops_ = 0 ;
        }

        slow_on_off_loops_++ ;
        if (slow_on_off_loops_ == kSlowLoopCount) {
            slow_on_off_ = !slow_on_off_ ;
            slow_on_off_loops_ = 0 ;
        }

        for(int i = 1 ; i <= max_leds_ ; i++) {
            boolean desired = false ;

            switch(led_state_[i - 1]) {
                case On:
                    desired = true ;
                    break ;
                case Off:
                    desired = false ;
                    break ;
                case SlowBlink:
                    desired = slow_on_off_ ;
                    break ;
                case FastBlink:
                    desired = fast_on_off_ ;
                    break ;
            }

            if (desired != led_onoff_[i - 1]) {
                hid_.setOutput(i, desired);
                led_onoff_[i - 1] = desired ;
            }
        }
    }
}
