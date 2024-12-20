package org.xero1425.subsystems.swerve;

import static edu.wpi.first.units.Units.Volts;

import java.util.Map;
import java.util.Optional;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import org.xero1425.base.HolonomicPathFollower;
import org.xero1425.base.ISubsystemSim;
import org.xero1425.base.LimelightHelpers;
import org.xero1425.base.XeroRobot;
import org.xero1425.base.LimelightHelpers.PoseEstimate;
import org.xero1425.math.Pose2dWithRotation;
import org.xero1425.math.XeroMath;
import org.xero1425.misc.SettingsValue;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.ApplyChassisSpeeds;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.generated.PathFollowingConstants;
import frc.robot.generated.SwerveConstants;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements
 * subsystem so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem, ISubsystemSim  {

    private static final String NAME = "swerve" ;

    private static final int kRecordModuleStates = (1  << 0) ;
    private static final int kRecordGyroYaw = (1 << 1) ;
    private static final int kDisplayRobotPose = (1 << 2) ;
    private static final int kDisplayEncoderValues = (1 << 3) ;
    private static final int kRecordModulePositions = (1 << 4) ;
    private static final int kRecordModuleTargets = (1 << 5) ;
    private int kDumpOutputSelected = kRecordModuleStates ;
    // private int kDumpOutputSelected =   kRecordModuleStates | 
    //                                     kDisplayAcquisition | 
    //                                     kDisplayEncoderValues | 
    //                                     kDisplayRobotPose |
    //                                     kRecordGyroYaw |
    //                                     kRecordModuleTargets ;

    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    private String limelight_name_ ;

    // Path follower for this drive base
    private HolonomicPathFollower follower_ ;    

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private final Rotation2d BlueAlliancePerspectiveRotation = Rotation2d.fromDegrees(0);

    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private final Rotation2d RedAlliancePerspectiveRotation = Rotation2d.fromDegrees(180);
    
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean hasAppliedOperatorPerspective = false;

    private XeroRobot robot_ ;

    //#region Characterization support
    private final SwerveRequest.SysIdSwerveTranslation TranslationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    // private final SwerveRequest.SysIdSwerveRotation RotationCharacterization = new SwerveRequest.SysIdSwerveRotation();
    // private final SwerveRequest.SysIdSwerveSteerGains SteerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();

    /* Use one of these sysidroutines for your particular test */
    private SysIdRoutine SysIdRoutineTranslation = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null,
                    Volts.of(4),
                    null,
                    (state) -> SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism(
                    (volts) -> setControl(TranslationCharacterization.withVolts(volts)),
                    null,
                    this));
    // private final SysIdRoutine SysIdRoutineRotation = new SysIdRoutine(
    //         new SysIdRoutine.Config(
    //                 null,
    //                 Volts.of(4),
    //                 null,
    //                 (state) -> SignalLogger.writeString("state", state.toString())),
    //         new SysIdRoutine.Mechanism(
    //                 (volts) -> setControl(RotationCharacterization.withVolts(volts)),
    //                 null,
    //                 this));
    // private final SysIdRoutine SysIdRoutineSteer = new SysIdRoutine(
    //         new SysIdRoutine.Config(
    //                 null,
    //                 Volts.of(7),
    //                 null,
    //                 (state) -> SignalLogger.writeString("state", state.toString())),
    //         new SysIdRoutine.Mechanism(
    //                 (volts) -> setControl(SteerCharacterization.withVolts(volts)),
    //                 null,
    //                 this));
    //#endregion

    /* Change this to the sysid routine you want to test */
    /* May be changed to SysIdRoutineRotation or SysIdRoutineSteer */

    private final SysIdRoutine RoutineToApply = SysIdRoutineTranslation;

    public CommandSwerveDrivetrain(XeroRobot robot, SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, 250, modules);

        robot_ = robot ;

        CommandScheduler.getInstance().registerSubsystem(this);
        robot.registerSubsystem(NAME, this);

        tareEverything();
        seedFieldRelative(new Pose2d(0, 0, Rotation2d.fromDegrees(180.0)));
        setVisionMeasurementStdDevs(VecBuilder.fill(0.7, 0.7, 9999999));

        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    public SettingsValue getProperty(String name) {
        return null ;
    }

    public void setLimelightName(String name) {
        limelight_name_ = name ;
    }

    public XeroRobot getRobot() {
        return robot_ ;
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier, String name) {
        Command cmd = run(() -> this.setControl(requestSupplier.get()));
        cmd.setName(name) ;
        return cmd ;
    }

    /*
     * Both the sysid commands are specific to one particular sysid routine, change
     * which one you're trying to characterize
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return RoutineToApply.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return RoutineToApply.dynamic(direction);
    }

    public ChassisSpeeds getCurrentRobotChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(getState().ModuleStates);
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }    

    @Override
    public void periodic() {

        startPeriodic();

        Command cmd = CommandScheduler.getInstance().requiring(this) ;
        if (cmd != null) {
            String str = cmd.getName() ;
            Logger.recordOutput("Command", str) ;
        }

        /* Periodically try to apply the operator perspective */
        /* If we haven't applied the operator perspective before, then we should apply it regardless of DS state */
        /* This allows us to correct the perspective in case the robot code restarts mid-match */
        /* Otherwise, only check and apply the operator perspective if the DS is disabled */
        /* This ensures driving behavior doesn't change until an explicit disable event occurs during testing*/
        if (!hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent((allianceColor) -> {
                this.setOperatorPerspectiveForward(
                        allianceColor == Alliance.Red ? RedAlliancePerspectiveRotation
                                : BlueAlliancePerspectiveRotation);
                hasAppliedOperatorPerspective = true;
            });
        }

        // //
        // // Feed the robot heading (e.g. gyro angle) to the limelight megatag2 algorithm
        // //
        Rotation2d robotHeading = getState().Pose.getRotation();
        LimelightHelpers.SetRobotOrientation(limelight_name_, robotHeading.getDegrees(), 0.0, 0.0, 0.0, 0.0, 0.0) ;

        // //
        // // Now, feed the limelight pose to the pose estimator to update our pose accuracy
        // //
        PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelight_name_) ;
        if (estimate != null && estimate.tagCount > 0 && m_angularVelocity.refresh().getValueAsDouble() < 360.0) {
            addVisionMeasurement(estimate.pose, estimate.timestampSeconds) ;
        }

        if (follower_ != null) {
            follower_.execute() ;
            if (!follower_.isDriving()) {
                follower_ = null ;
            }
        }

        dumpOutput() ;

        endPeriodic();
    }

    public void startPeriodic() {
        robot_.startPeriodic(NAME) ;
    }

    public void endPeriodic() {
        robot_.endPeriodic(NAME) ;
    }

    public void driveTo(String pathname, Pose2d[] imd, Pose2dWithRotation dest, double maxv, double maxa, double pre_rot_time, double post_rot_time, double to) {
        follower_ = new HolonomicPathFollower(createHolonimicPathFollowerConfig());
        follower_.driveTo(pathname, imd, dest, maxv, maxa, pre_rot_time, post_rot_time, to);
    }

    public void stopPath() {

        setControl(new ApplyChassisSpeeds().withSpeeds(new ChassisSpeeds())) ;
        follower_ = null ;
    }

    public double getPathDistance() {
        double ret = 0.0 ;

        if (follower_ != null) {
            ret = follower_.getDistance() ;
        }

        return ret ;
    }

    public boolean isFollowingPath() {
        return follower_ != null ;
    }

    private void dumpOutput() {
        if ((kDumpOutputSelected & kRecordModuleStates) == kRecordModuleStates) {
            var states = m_cachedState.ModuleStates ;
            Logger.recordOutput("swerve:states", states) ;
        }

        if ((kDumpOutputSelected & kRecordModuleTargets) == kRecordModuleTargets) {
            var states = m_cachedState.ModuleTargets ;
            Logger.recordOutput("swerve:targets", states) ;
        }        

        if ((kDumpOutputSelected & kRecordGyroYaw) == kRecordGyroYaw) {
            Logger.recordOutput("yaw", XeroMath.normalizeAngleDegrees(getPigeon2().getYaw().getValueAsDouble())) ;
        }

        if ((kDumpOutputSelected & kDisplayRobotPose) == kDisplayRobotPose) {        
            SmartDashboard.putNumber("db-x", getState().Pose.getX()) ;
            SmartDashboard.putNumber("db-y", getState().Pose.getY()) ;
            SmartDashboard.putNumber("db-a", getState().Pose.getRotation().getDegrees()) ;  
        }

        if ((kDumpOutputSelected & kDisplayEncoderValues) == kDisplayEncoderValues) {
            SmartDashboard.putNumber("fl-enc", this.Modules[0].getDriveMotor().getPosition().getValueAsDouble()) ;
            SmartDashboard.putNumber("fr-enc", this.Modules[1].getDriveMotor().getPosition().getValueAsDouble()) ;
            SmartDashboard.putNumber("bl-enc", this.Modules[2].getDriveMotor().getPosition().getValueAsDouble()) ;
            SmartDashboard.putNumber("br-enc", this.Modules[3].getDriveMotor().getPosition().getValueAsDouble()) ;            
        }

        if ((kDumpOutputSelected & kRecordModulePositions) == kRecordModulePositions) {
            var states = m_modulePositions ;
            Logger.recordOutput("mp:fl-a", XeroMath.normalizeAngleDegrees(states[0].angle.getDegrees())) ;
            Logger.recordOutput("mp:fr-a", XeroMath.normalizeAngleDegrees(states[1].angle.getDegrees())) ;
            Logger.recordOutput("mp:bl-a", XeroMath.normalizeAngleDegrees(states[2].angle.getDegrees())) ;
            Logger.recordOutput("mp:br-a", XeroMath.normalizeAngleDegrees(states[3].angle.getDegrees())) ;
            Logger.recordOutput("mp:fl-v", states[0].distanceMeters) ;
            Logger.recordOutput("mp:fr-v", states[1].distanceMeters) ;
            Logger.recordOutput("mp:bl-v", states[2].distanceMeters) ;
            Logger.recordOutput("mp:br-v", states[3].distanceMeters) ;
        }        
    }

    private HolonomicPathFollower.Config createHolonimicPathFollowerConfig() {
        HolonomicPathFollower.Config cfg = new HolonomicPathFollower.Config() ;

        cfg.max_rot_velocity = SwerveConstants.kMaxRotationalSpeed ;
        cfg.max_rot_acceleration = SwerveConstants.kMaxRotationalAccel ;

        cfg.rot_p = PathFollowingConstants.RotCtrl.kP ;
        cfg.rot_i = PathFollowingConstants.RotCtrl.kI ;
        cfg.rot_d = PathFollowingConstants.RotCtrl.kD ;

        cfg.x_d = PathFollowingConstants.XCtrl.kD ;
        cfg.x_i = PathFollowingConstants.XCtrl.kI ;
        cfg.x_p = PathFollowingConstants.XCtrl.kP ;

        cfg.y_d = PathFollowingConstants.YCtrl.kD ;
        cfg.y_i = PathFollowingConstants.YCtrl.kI ;
        cfg.y_p = PathFollowingConstants.YCtrl.kP ;

        cfg.xytolerance = PathFollowingConstants.kXYTolerance ;
        cfg.rot_tolerance = Rotation2d.fromDegrees(PathFollowingConstants.kAngleTolerance) ;

        cfg.pose_supplier = () -> getState().Pose ;
        cfg.output_consumer = (ChassisSpeeds spd) -> { setControl(new ApplyChassisSpeeds().withSpeeds(spd)) ; } ;

        return cfg ;
    }    

    public Map<String, TalonFX> getCTREMotors() {
        return null ;
    }

    public void yandbPressed() {
        Optional<Alliance> alliance = DriverStation.getAlliance() ;
        if (alliance.isPresent()) {
            Pose2d pose  ;
            if (alliance.get() == Alliance.Red) {
                pose = new Pose2d(0, 0, Rotation2d.fromDegrees(180.0)) ;
            }
            else {
                pose = new Pose2d(0, 0, Rotation2d.fromDegrees(0.0)) ;                
            }
            seedFieldRelative(pose) ;
        }
        else {
            DriverStation.reportError("Gamepad Y & B pressed before alliance is known (should be impossible)", false) ;
        }
    }
}

