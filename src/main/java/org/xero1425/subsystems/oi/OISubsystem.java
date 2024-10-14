package org.xero1425.subsystems.oi;

import java.util.HashMap;
import java.util.Map;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;
import org.xero1425.base.XeroRobot;
import org.xero1425.base.XeroSubsystem;
import org.xero1425.misc.SettingsValue;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public abstract class OISubsystem extends XeroSubsystem {
    public enum LEDState {
        On,
        Off,
        SlowBlink,
        FastBlink
    }

    final private double SlowFactor = 0.1 ;
    final private int MaxOIButtons = 32 ;
    final private int MaxLEDs = 16 ;

    private OIInputsOutputs ios_ ;
    private OIInputsAutoLogged inputs_ ;

    private boolean enabled_ ;
    private CommandXboxController ctrl_ ;
    private int oiport_ ;
    private GenericHID oihid_ ;

    private boolean isRumbling ;
    private double rumble_ ;
    private double stop_rumble_time_ ;

    private HashMap<String, Integer> buttons_ ;
    private HashMap<String, Trigger> triggers_ ;

    public OISubsystem(XeroRobot robot, String name, int gp, int oi) {
        super(robot, name) ;

        oihid_ = new GenericHID(oi) ;

        ios_ = new OIInputsOutputsHID(oihid_, MaxOIButtons, MaxLEDs) ;
        inputs_ = new OIInputsAutoLogged() ;
        
        ctrl_ = new CommandXboxController(gp) ;

        oiport_ = oi ;
        enabled_ = true ;
        isRumbling = false ;

        buttons_ = new HashMap<>();
    }

    public void enable() {
        enabled_ = true ;
    }

    public void disable() {
        enabled_ = false ;
    }

    public void setLEDState(int led, LEDState st) {
        ios_.setLEDState(led, st) ;
    }

    @Override
    public void periodic() {
        startPeriodic();

        ios_.updateInputs(inputs_);
        Logger.processInputs("oi", inputs_);
        
        if (isRumbling) {
            if (Timer.getFPGATimestamp() > stop_rumble_time_) {
                rumble_ = 0.0 ;
                isRumbling = false ;
                getXBoxHIDDevice().setRumble(RumbleType.kBothRumble, 0.0);
            }
        }

        String gp = getGamePadButtonString() ;
        String oi = getOIButtonString() ;

        Logger.recordOutput("oi:gamepad", "[" + gp + "]") ;
        Logger.recordOutput("oi:oibuttons", "[" + oi + "]") ;

        ios_.updateLEDs() ;

        endPeriodic();
    }

    public Map<String, TalonFX> getCTREMotors() {
        return null ;
    }    

    public SettingsValue getProperty(String name) {
        return null ;
    }    

    public void setRumble(double value, double duration) {
        rumble_ = value ;
        isRumbling = true ;
        stop_rumble_time_ = Timer.getFPGATimestamp() + duration ;
        getXBoxHIDDevice().setRumble(RumbleType.kBothRumble, value);
    }

    public double getRumble() {
        return rumble_ ;
    }    

    public CommandXboxController getXBoxController() {
        return ctrl_ ;
    }

    public XboxController getXBoxHIDDevice() {
        return ctrl_.getHID() ;
    }

    public double getLeftX() {
        return getStickValue(()->ctrl_.getHID().getLeftX()) ;
    }

    public double getLeftY() {
        return getStickValue(()->ctrl_.getHID().getLeftY()) ;
    }

    public double getRightX() {
        return getStickValue(()->ctrl_.getHID().getRightX()) ;
    }

    public double getRightY() {
        return getStickValue(()->ctrl_.getHID().getRightY()) ;
    }    

    protected int getOIPort() {
        return oiport_ ;
    }

    protected String getOIButtonString() {
        String ret = "" ;

        for(String key : buttons_.keySet()) {
            int button = buttons_.get(key) ;
            if (inputs_.buttons_[button]) {
                if (ret.length() > 0) {
                    ret += "," ;
                }
                ret += key ;
            }
        }
        return ret ;
    }

    private String findFunctionByButton(int button) {
        String func = null ;

        for(String key : buttons_.keySet()) {
            if (buttons_.get(key) == button) {
                func = key ;
                break ;
            }
        }

        return func ;
    }

    protected void mapButton(String name, int button) throws Exception {
        if (button > MaxOIButtons) {
            throw new Exception("button number (" + button + ") is invalid - must be less than or equal to " + MaxOIButtons) ;
        }

        String func = findFunctionByButton(button) ;
        if (func != null) {
            throw new Exception("button number (" + button + ") has already been bound to function '" + func + "'") ;
        }

        buttons_.put(name, button) ;
        triggers_.put(name, new Trigger(()->inputs_.buttons_[button - 1])) ;
    }

    public Trigger getTrigger(String name) {
        return triggers_.get(name) ;
    }

    private double getStickValue(DoubleSupplier supplier) {
        double ret = 0.0 ;

        if (enabled_) {
            ret = supplier.getAsDouble() ;

            if (getXBoxHIDDevice().getXButton()) {
                ret = ret * SlowFactor ;
            }
            else {
                ret = Math.signum(ret) * ret * ret;
            }
        }
            
        return ret ;
    }

    private String getGamePadButtonString() {
        String str = "" ;

        if (getXBoxHIDDevice().getAButton()) {
            if (str.length() > 0)
                str += "," ;
            str += "a" ;
        }

        if (getXBoxHIDDevice().getBButton()) {
            if (str.length() > 0)
                str += "," ;
            str += "b" ;
        }        

        if (getXBoxHIDDevice().getXButton()) {
            if (str.length() > 0)
                str += "," ;
            str += "x" ;
        }    
        
        if (getXBoxHIDDevice().getYButton()) {
            if (str.length() > 0)
                str += "," ;
            str += "y" ;
        }   
        
        if (getXBoxHIDDevice().getBackButton()) {
            if (str.length() > 0)
                str += "," ;
            str += "back" ;
        }      
        
        if (getXBoxHIDDevice().getStartButton()) {
            if (str.length() > 0)
                str += "," ;
            str += "start" ;
        }   
        
        if (getXBoxHIDDevice().getLeftBumper()) {
            if (str.length() > 0)
                str += "," ;
            str += "lb" ;
        }     
        
        if (getXBoxHIDDevice().getRightBumper()) {
            if (str.length() > 0)
                str += "," ;
            str += "rb" ;
        }          

        if (getXBoxHIDDevice().getLeftStickButton()) {
            if (str.length() > 0)
                str += "," ;
            str += "ls" ;
        }     
        
        if (getXBoxHIDDevice().getRightStickButton()) {
            if (str.length() > 0)
                str += "," ;
            str += "rs" ;
        }    

        return str ;
    }

}
