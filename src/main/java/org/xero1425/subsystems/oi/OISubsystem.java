package org.xero1425.subsystems.oi;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class OISubsystem extends SubsystemBase {
    final private double SlowFactor = 0.1 ;

    private boolean enabled_ ;
    private CommandXboxController ctrl_ ;
    private int oiport_ ;

    private boolean isRumbling ;
    private double rumble_ ;
    private double stop_rumble_time_ ;

    public OISubsystem(int gp, int oi) {
        ctrl_ = new CommandXboxController(gp) ;
        oiport_ = oi ;
        enabled_ = true ;
        isRumbling = false ;
    }

    @Override
    public void periodic() {

        if (isRumbling) {
            if (Timer.getFPGATimestamp() > stop_rumble_time_) {
                rumble_ = 0.0 ;
                isRumbling = false ;
                getXBoxHIDDevice().setRumble(RumbleType.kBothRumble, 0.0);
            }
        }          
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
}
