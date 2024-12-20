package org.xero1425.base;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.xero1425.misc.SimArgs;
import org.xero1425.simulator.engine.SimulationEngine;
import org.xero1425.subsystems.oi.OISubsystem;
import org.xero1425.subsystems.swerve.CommandSwerveDrivetrain;
import org.xero1425.subsystems.swerve.Telemetry;
import org.xero1425.subsystems.vision.AprilTagVisionSubsystem;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.BuildConstants;
import frc.robot.generated.SwerveConstants;
import frc.robot.generated.TunerConstantsCompetition;
import frc.robot.generated.TunerConstantsPractice;

public abstract class XeroRobot extends LoggedRobot {

    private static XeroRobot robot_ = null ;

    private XeroAutoCommand auto_mode_ ;
    private List<XeroAutoCommand> automodes_ ;
    private SendableChooser<XeroAutoCommand> chooser_ ;
    private GenericEntry chosen_ ;
    private GenericEntry desc_ ;
    private AprilTagFieldLayout layout_ ;
    private boolean auto_modes_created_ ;

    protected CommandSwerveDrivetrain db_ ;
    private SwerveRequest.SwerveDriveBrake brake_ ;
    private SwerveRequest.RobotCentric pov_move_;
    private SwerveRequest.FieldCentric drive_ ;

    protected AprilTagVisionSubsystem vision_;

    protected OISubsystem oi_ ;

    private String char_subsystem_ ;
    private String char_motor_ ;

    private HashMap<String, Double> periodic_times_ ;
    private HashMap<String, ISubsystemSim> subsystems_ ;

    //
    // Telemetry related
    //
    private Telemetry telemetry_ ;

    public XeroRobot(boolean logToNetworkTables) {
        if (robot_ != null) {
            throw new RuntimeException("XeroRobot is a singleton class") ;
        }
        robot_ = this ;
        subsystems_ = new HashMap<>() ;
        periodic_times_ = new HashMap<>() ;

        setFieldLayout(AprilTagFields.kDefaultField.loadAprilTagLayoutField()) ;

        drive_ = null ;
        pov_move_ = null ;
        brake_ = null ;

        auto_modes_created_ = false ;
        automodes_ = new ArrayList<>() ;
        auto_mode_ = null;

        enableAdvantageKitLogger(logToNetworkTables) ;

        if (RobotBase.isSimulation()) {
            String str = SimArgs.InputFileName;
            if (str == null)
                str = getSimulationFileName() ;

            if (str == null) {
                System.out.println("The code is setup to simulate, but the derived robot class did not provide a stimulus file") ;
                System.out.println("Not initializing the Xero1425 Simulation engine - assuming Romi robot") ;
            }
            else {
                SimulationEngine.initializeSimulator(this);
                addRobotSimulationModels() ;
                SimulationEngine.getInstance().initAll(str) ;
            }
        }
    }

    public abstract boolean isCharMode() ;
    public abstract String  getSimulationFileName() ;
    public abstract String getSimulationAutoMode() ;
    protected abstract String getName() ;
    protected abstract String getPracticeSerialNumber() ;
    protected abstract void createCompetitionAutoModes() ;
    protected abstract void addRobotSimulationModels() ;
    protected abstract void robotSpecificBindings() ;
    protected abstract String getCharSubsystem() ;
    protected abstract String getCharMotor() ;

    protected abstract OISubsystem createOISubsystem() ;
    protected abstract AprilTagVisionSubsystem createVisionSubsystem();

    public OISubsystem getOISubsystem() {
        return oi_ ;
    }

    public CommandSwerveDrivetrain getDriveBase() {
        return db_ ;
    }

    public void startPeriodic(String name) {
        periodic_times_.put(name, Timer.getFPGATimestamp()) ;
    }

    public void endPeriodic(String name) {
        double runtime = Double.NaN;
        if (periodic_times_.containsKey(name)) {
            runtime = Timer.getFPGATimestamp() - periodic_times_.get(name) ;
        }

        Logger.recordOutput("timingsMS:" + name, 1000 * runtime) ;
    }

    public ISubsystemSim getSubsystemByName(String name) {
        return subsystems_.get(name) ;
    }

    public void robotInit() {
        super.robotInit() ;

        if (RobotBase.isSimulation() && SimulationEngine.getInstance() != null)
        {
            //
            // If we are simulating, create the simulation modules required
            //
            SimulationEngine.getInstance().createModels() ;
        }

        //
        // Create the OI
        //
        oi_ = createOISubsystem() ;

        //
        // Create the drive base
        //
        createDriveBase() ;

        /**
         * Create Vision Subsystem
         */
        vision_ = createVisionSubsystem();

        char_subsystem_ = getCharSubsystem() ;
        char_motor_ = getCharMotor() ;

        if (isCharMode()) {
            if (char_subsystem_ != null && char_motor_ != null) {
                charBindings(char_subsystem_, char_motor_) ;
            }
        }
        else {
            standardDriveBaseBindings();
            robotSpecificBindings() ;
        }

        db_.registerTelemetry(telemetry_::telemeterize) ;
    }

    private void charBindings(String subsystem, String motor) {
        try {
            XeroSubsystem subsys = (XeroSubsystem)getSubsystemByName(subsystem) ;
            Command fquasi = subsys.getQuasistatic(motor, Direction.kForward) ;
            Command rquasi = subsys.getQuasistatic(motor, Direction.kReverse) ;
            Command fdynamic = subsys.getDynamic(motor, Direction.kForward) ;
            Command rdynamic = subsys.getDynamic(motor, Direction.kReverse) ;

            if (fquasi != null && rquasi != null && fdynamic != null && rdynamic != null) {
                oi_.getXBoxController().leftBumper().and(oi_.getXBoxController().rightBumper()).and(oi_.getXBoxController().x()).whileTrue(fquasi);
                oi_.getXBoxController().leftBumper().and(oi_.getXBoxController().rightBumper()).and(oi_.getXBoxController().y()).whileTrue(rquasi);
                oi_.getXBoxController().leftBumper().and(oi_.getXBoxController().rightBumper()).and(oi_.getXBoxController().a()).whileTrue(fdynamic);
                oi_.getXBoxController().leftBumper().and(oi_.getXBoxController().rightBumper()).and(oi_.getXBoxController().b()).whileTrue(rdynamic);
            }
        }
        catch(Exception ex) {
        }
    }

    private void standardDriveBaseBindings() {
        //
        // Create the standard bindings between the gamepad and the drive base
        //
        db_.setDefaultCommand(
            db_.applyRequest(() -> drive_.withVelocityX(oi_.getLeftY() * TunerConstantsCompetition.kSpeedAt12VoltsMps)
                                         .withVelocityY(oi_.getLeftX() * TunerConstantsCompetition.kSpeedAt12VoltsMps)
                                         .withRotationalRate(oi_.getRightX() * SwerveConstants.kMaxRotationalSpeed), "drive"
                            ).ignoringDisable(true));

        oi_.getXBoxController().y().and(oi_.getXBoxController().b()).onTrue(db_.runOnce(()->db_.yandbPressed()).ignoringDisable(true)) ;
        oi_.getXBoxController().leftBumper().whileTrue(db_.applyRequest(() -> brake_, "brake").ignoringDisable(true)) ;

        oi_.getXBoxController().pov(0).whileTrue(db_.applyRequest(() -> pov_move_.withVelocityX(0.5).withVelocityY(0), "pov0")) ;
        oi_.getXBoxController().pov(90).whileTrue(db_.applyRequest(() -> pov_move_.withVelocityX(0.0).withVelocityY(-0.5), "pov90")) ;
        oi_.getXBoxController().pov(180).whileTrue(db_.applyRequest(() -> pov_move_.withVelocityX(-0.5).withVelocityY(0), "pov180")) ;
        oi_.getXBoxController().pov(270).whileTrue(db_.applyRequest(() -> pov_move_.withVelocityX(0.0).withVelocityY(0.5), "pov270")) ;
    }

    private void createDriveBase() {

        if (isPracticeBot()) {
            //
            // Create the drivebase
            //
            db_ = new CommandSwerveDrivetrain(this,
                                            TunerConstantsPractice.DrivetrainConstants,
                                            TunerConstantsPractice.FrontLeft,
                                            TunerConstantsPractice.FrontRight,
                                            TunerConstantsPractice.BackLeft,
                                            TunerConstantsPractice.BackRight);


            telemetry_ = new Telemetry(TunerConstantsPractice.kSpeedAt12VoltsMps) ;

            drive_ = new SwerveRequest.FieldCentric()
                            .withDeadband(TunerConstantsPractice.kSpeedAt12VoltsMps * 0.05)
                            .withRotationalDeadband(SwerveConstants.kMaxRotationalSpeed * 0.05)
                            .withDriveRequestType(DriveRequestType.Velocity);
        }
        else {
            //
            // Create the drivebase
            //
            db_ = new CommandSwerveDrivetrain(this,
                                            TunerConstantsCompetition.DrivetrainConstants,
                                            TunerConstantsCompetition.FrontLeft,
                                            TunerConstantsCompetition.FrontRight,
                                            TunerConstantsCompetition.BackLeft,
                                            TunerConstantsCompetition.BackRight);


            telemetry_ = new Telemetry(TunerConstantsCompetition.kSpeedAt12VoltsMps) ;

            drive_ = new SwerveRequest.FieldCentric()
                            .withDeadband(TunerConstantsCompetition.kSpeedAt12VoltsMps * 0.05)
                            .withRotationalDeadband(SwerveConstants.kMaxRotationalSpeed * 0.05)
                            .withDriveRequestType(DriveRequestType.Velocity);
        }

        brake_ = new SwerveRequest.SwerveDriveBrake() ;
        pov_move_ = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    }

    public AprilTagFieldLayout getFieldLayout() {
        return layout_ ;
    }

    protected void setFieldLayout(AprilTagFieldLayout layout) {
        layout_ = layout ;
    }

    protected void createAutoModes() {
        if (DriverStation.isDSAttached()) {
            if (!auto_modes_created_) {
                createCompetitionAutoModes() ;
                auto_modes_created_ = true ;
            }

            autoModeChooser();
        }
    }

    protected void addAutoMode(XeroAutoCommand mode) {
        automodes_.add(mode) ;
    }

    private void enableAdvantageKitLogger(boolean logToNetworkTables) {
        Logger.disableDeterministicTimestamps();

        if (XeroRobot.isSimulation() || logToNetworkTables) {
            Logger.addDataReceiver(new NT4Publisher());
        }
        Logger.addDataReceiver(new WPILOGWriter()) ;

        Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
        Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
        Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
        Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);

        Logger.start() ;
    }

    @Override
    public void robotPeriodic() {
        //
        // Runs the Scheduler.
        //
        CommandScheduler.getInstance().run();

        if (isSimulation()) {
            SimulationEngine engine = SimulationEngine.getInstance() ;
            if (engine != null) {
                engine.run(getPeriod());
            }
        }

    }

    public void registerSubsystem(String name, ISubsystemSim subsystem) {
        subsystems_.put(name, subsystem) ;
    }

    @Override
    public void autonomousInit() {
        super.autonomousInit() ;

        if (auto_mode_ != null) {
            Logger.recordMetadata("auto-mode-run", auto_mode_.getName()) ;
            auto_mode_.schedule() ;
        }
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void simulationInit() {
    }

    @Override
    public void simulationPeriodic() {
    }

    @Override
    public void disabledPeriodic() {
        createAutoModes();
    }

    protected void enableMessages() {
    }

    public boolean isPracticeBot() {
        return RobotController.getSerialNumber().equals(getPracticeSerialNumber()) ;
    }

    public boolean isCompetitionBot() {
        return RobotBase.isReal() && !isPracticeBot() ;
    }

    public double getTime() {
        return Timer.getFPGATimestamp() ;
    }

    private void autoModeChanged(XeroAutoCommand mode) {
        chosen_.setString(mode.toString()) ;
        desc_.setString(mode.getDescription()) ;
    }

    private void autoModeChooser() {
        if (XeroRobot.isSimulation()) {
            String name = getSimulationAutoMode() ;
            for (XeroAutoCommand cmd: automodes_) {
                if (cmd.getName().equals(name)) {
                    auto_mode_ = cmd ;
                    break ;
                }
            }
        } else {
            if (chooser_ == null && automodes_.size() > 0) {
                chooser_ = new SendableChooser<>();
                chooser_.onChange((mode) -> autoModeChanged(mode)) ;
                boolean defaultSet = false ;
                for (XeroAutoCommand mode : automodes_) {
                    chooser_.addOption(mode.toString(), mode) ;
                    if (!defaultSet) {
                        auto_mode_ = mode ;
                        chooser_.setDefaultOption(mode.toString(), mode) ;
                        defaultSet = true ;
                    }
                }

                ShuffleboardTab tab = Shuffleboard.getTab("AutoMode") ;
                chosen_ = tab.add("Auto Mode Selected", auto_mode_.toString()).withSize(2, 1).withPosition(3, 0).getEntry() ;
                desc_ = tab.add("Auto Mode Description", auto_mode_.toString()).withSize(5, 2).withPosition(0, 1).getEntry() ;
                tab.add("Auto Mode Selecter", chooser_).withSize(2,1).withWidget(BuiltInWidgets.kComboBoxChooser).withPosition(0, 0) ;
            }

            if (chooser_ != null) {
                auto_mode_ = chooser_.getSelected() ;
            }
        }
    }
}
