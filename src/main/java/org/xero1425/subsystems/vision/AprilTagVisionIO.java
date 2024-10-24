package org.xero1425.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;
import org.xero1425.struct.vision.XeroFiducial;
import org.xero1425.struct.vision.XeroPoseEstimate;

import edu.wpi.first.math.geometry.Translation2d;

public interface AprilTagVisionIO {

    @AutoLog
    public class AprilTagVisionIOInputs {
        // General Values That Will be Replayed
        public int simpleID = 0;
        public double simpleX = 0.0;
        public double simpleY = 0.0;
        public double simpleArea = 0.0;
        public boolean simpleValid = false;
        
        public Translation2d[] rawCorners = new Translation2d[] {};
        public XeroFiducial[] fiducials = new XeroFiducial[] {};

        public XeroPoseEstimate poseEstimate = new XeroPoseEstimate();
    }

    /**
     * Updates the inputs object with values from the hardware.
     * @param inputs The inputs to update
     */
    public default void updateInputs(AprilTagVisionIOInputsAutoLogged inputs) {};

    /**
     * Forces the indicator light on the camera to be off.
     */
    public default void forceOff() {};

    /**
     * Forces the indicator light on the camera to blink.
     */
    public default void forceBlink() {};

    /**
     * Forces the indicator light on the camera to be on.
     */
    public default void forceOn() {};

    /**
     * Resets the indicator light on the camera to be controlled by its own software/pipelines.
     */
    public default void resetLed() {};

    /**
     * Gives the robot orientation to the camera for odometry.
     * Blue origion, CCW-positive, 0 degrees facing red alliance wall
     */
    public default void giveRobotOrientation(double yaw, double yawRate, double pitch, double pitchRate, double roll, double rollRate) {};

}