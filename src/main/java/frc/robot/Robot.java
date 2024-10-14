// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.xero1425.base.XeroRobot;
import org.xero1425.subsystems.oi.OISubsystem;

import frc.robot.subsystems.AllegroOISubsystem;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends XeroRobot {
    private final static boolean kLogToNetworkTables = true ;

    public Robot() {
        super(kLogToNetworkTables) ;
    }

    /**
     * This function is run when the robot is first started up and should be used
     * for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        //
        // The base class will create the drive base and the drive controller.
        // It assumes an XBox controller for now.
        //
        super.robotInit() ;

        //
        // TODO: Create other subsystems here
        //
    }

    public String getName() {
        return "TemplateRobot" ;
    }

    public boolean isCharMode() {
        return false ;
    }

    @Override
    public String getPracticeSerialNumber() {
        return null;
    }

    @Override
    public void createCompetitionAutoModes() {
    }    

    @Override
    public String getSimulationFileName() {
        return null ;
    }

    @Override
    public String getSimulationAutoMode() {
        return null ;
    }

    @Override
    public void addRobotSimulationModels() {
    }

    @Override
    protected OISubsystem createOISubsystem() {
        return new AllegroOISubsystem(this) ;
    }
}
