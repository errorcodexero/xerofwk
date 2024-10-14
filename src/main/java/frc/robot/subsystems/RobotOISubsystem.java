package frc.robot.subsystems;

import org.xero1425.base.XeroRobot;
import org.xero1425.subsystems.oi.OISubsystem;

public class RobotOISubsystem extends OISubsystem {
    public RobotOISubsystem(XeroRobot robot) {
        super(robot, "OI", OIConstants.kDriverControllerPort, OIConstants.kOIControllerPort) ;
    }
}
