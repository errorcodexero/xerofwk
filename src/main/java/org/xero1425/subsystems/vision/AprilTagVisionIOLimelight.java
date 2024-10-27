package org.xero1425.subsystems.vision;

import java.util.ArrayList;

import org.xero1425.base.LimelightHelpers;
import org.xero1425.base.LimelightHelpers.LimelightResults;
import org.xero1425.base.LimelightHelpers.RawDetection;
import org.xero1425.struct.vision.XeroFiducial;
import org.xero1425.struct.vision.XeroPoseEstimate;

import edu.wpi.first.math.geometry.Translation2d;

public class AprilTagVisionIOLimelight implements AprilTagVisionIO {

    private final String name_;

    /**
     * Creates a new Limelight implementation, this implementation is using the Limelight Lib with a Limelight.
     * This specifies a name.
     * @param name The name of the limelight.
     */
    public AprilTagVisionIOLimelight(String name) {
        name_ = name;
    }

    @Override
    public void forceOff() {
        LimelightHelpers.setLEDMode_ForceOff(name_);
    }
    
    @Override
    public void forceBlink() {
        LimelightHelpers.setLEDMode_ForceBlink(name_);
    }

    @Override
    public void forceOn() {
        LimelightHelpers.setLEDMode_ForceOn(name_);
    }

    @Override
    public void resetLed() {
        LimelightHelpers.setLEDMode_PipelineControl(name_);
    }

    @Override
    public void updateInputs(AprilTagVisionIOInputsAutoLogged inputs) {
        inputs.simpleX = LimelightHelpers.getTX(name_);
        inputs.simpleY = LimelightHelpers.getTY(name_);
        inputs.simpleArea = LimelightHelpers.getTA(name_);
        inputs.simpleValid = LimelightHelpers.getTV(name_);
        inputs.simpleID = (int) LimelightHelpers.getFiducialID(name_);

        RawDetection[] detections = LimelightHelpers.getRawDetections(name_);
        ArrayList<Translation2d> corners = new ArrayList<>();

        for (RawDetection detection : detections) {
            corners.add(new Translation2d(detection.corner0_X, detection.corner0_Y));
            corners.add(new Translation2d(detection.corner1_X, detection.corner1_Y));
            corners.add(new Translation2d(detection.corner2_X, detection.corner2_Y));
            corners.add(new Translation2d(detection.corner3_X, detection.corner3_Y));
        }

        inputs.rawCorners = corners.toArray(new Translation2d[0]);
        
        LimelightResults results = LimelightHelpers.getLatestResults(name_);
        inputs.fiducials = XeroFiducial.fromLimelightArray(results.targets_Fiducials);

        inputs.poseEstimate = XeroPoseEstimate.of(LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name_));
    }

    @Override
    public void giveRobotOrientation(double yaw, double yawRate, double pitch, double pitchRate, double roll, double rollRate) {
        LimelightHelpers.SetRobotOrientation(name_, yaw, 0, 0, 0, 0, 0);
    }

}