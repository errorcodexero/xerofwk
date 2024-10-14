package frc.robot.commands.automodes.test;

import org.xero1425.base.XeroAutoCommand;
import org.xero1425.base.XeroRobot;
import org.xero1425.subsystems.oi.OISetLEDStateCommand;
import org.xero1425.subsystems.oi.OISubsystem;
import org.xero1425.subsystems.oi.OISubsystem.LEDState;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class LEDTestAutoModeCommand extends XeroAutoCommand {
    private SequentialCommandGroup list_ ;

    public LEDTestAutoModeCommand(XeroRobot robot) {
        super(robot, "LEDTest", "Test the LED capabilities") ;

        OISubsystem oi = robot.getOISubsystem() ;
        WaitCommand delay = new WaitCommand(5.0) ;
        OISetLEDStateCommand ledon = new OISetLEDStateCommand(oi, 1, LEDState.SlowBlink) ;
        OISetLEDStateCommand ledoff = new OISetLEDStateCommand(oi, 1, LEDState.Off) ;        
        list_ = new SequentialCommandGroup(ledon, delay, ledoff) ;
    }

    @Override
    public void initialize() {
        CommandScheduler.getInstance().schedule(list_);
    }

    @Override
    public void end(boolean b) {
        if (!list_.isFinished()) {
            list_.cancel();
        }
    }

    @Override
    public boolean isFinished() {
        return list_.isFinished() ;
    }
}
