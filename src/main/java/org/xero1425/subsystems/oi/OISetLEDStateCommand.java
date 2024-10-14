package org.xero1425.subsystems.oi;

import edu.wpi.first.wpilibj2.command.Command;

public class OISetLEDStateCommand extends Command {
    private OISubsystem oi_ ;
    private int led_ ;
    private OISubsystem.LEDState state_ ;

    public OISetLEDStateCommand(OISubsystem oi, int index, OISubsystem.LEDState st) {
        oi_ = oi ;
        led_ = index ;
        state_ = st ;
    }

    @Override
    public void initialize() {
        oi_.setLEDState(led_, state_);
    }

    @Override
    public boolean isFinished() {
        return true ;
    }
}
