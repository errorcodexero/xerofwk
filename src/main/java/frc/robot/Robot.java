// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.photonvision.simulation.SimCameraProperties;
import org.xero1425.base.XeroRobot;
import org.xero1425.simulator.engine.ModelFactory;
import org.xero1425.simulator.engine.SimulationEngine;
import org.xero1425.subsystems.oi.OISubsystem;
import org.xero1425.subsystems.vision.AprilTagVisionIO;
import org.xero1425.subsystems.vision.AprilTagVisionIOLimelight;
import org.xero1425.subsystems.vision.AprilTagVisionIOSim;
import org.xero1425.subsystems.vision.AprilTagVisionSubsystem;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.commands.automodes.test.LEDTestAutoModeCommand;
import frc.robot.subsystems.RobotOISubsystem;

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
        addAutoMode(new LEDTestAutoModeCommand(this));
    }

    @Override
    public String getSimulationFileName() {
        return "oiled" ;
    }

    @Override
    public String getSimulationAutoMode() {
        return "LEDTest" ;
    }

    @Override
    public void addRobotSimulationModels() {
        ModelFactory factory = SimulationEngine.getInstance().getModelFactory();
        factory.registerModel("robot-oi", "frc.models.RobotOIModel");         
    }

    @Override
    protected OISubsystem createOISubsystem() {
        return new RobotOISubsystem(this) ;
    }

    @Override
    protected AprilTagVisionSubsystem createVisionSubsystem() {
        AprilTagVisionIO visionio;

        if (isReal()) {
            visionio = new AprilTagVisionIOLimelight("limelight");
        } else {
            visionio = new AprilTagVisionIOSim(
                "sim", // Sim name
                () -> db_.getState().Pose, // Pose supplier
                getFieldLayout(), // Field layout
                new Transform3d( // Robot-Relative Camera Pose
                    new Translation3d(-0.3549, 0, 0.16),
                    new Rotation3d(0, Units.degreesToRadians(-40), Units.degreesToRadians(180))
                ),
                SimCameraProperties.LL2_960_720() // Camera properties
            );
        }

        return new AprilTagVisionSubsystem(
            visionio,
            getFieldLayout(), // Field Layout
            () -> db_.getState().Pose, // Pose supplier
            estimate -> { // Pose estimate fusing
                db_.addVisionMeasurement(
                    estimate.pose,
                    estimate.timestamp,
                    VecBuilder.fill(0.7, 0.7, 999999)
                );
            }
        );
    }

    @Override
    protected void robotSpecificBindings() {
    }

    @Override
    protected String getCharSubsystem() {
        return null ;
    }

    protected String getCharMotor() {
        return null ;
    }
}
