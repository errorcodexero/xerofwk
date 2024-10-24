package org.xero1425.struct.vision;

import org.xero1425.base.LimelightHelpers.PoseEstimate;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.struct.StructSerializable;

public class XeroPoseEstimate implements StructSerializable {

    public final Pose2d pose;
    public final double timestamp;
    public final int tagCount;
    public final boolean valid;

    public XeroPoseEstimate(Pose2d pose, double timestamp, int tagCount, boolean valid) {
        this.pose = pose;
        this.timestamp = timestamp;
        this.tagCount = tagCount;
        this.valid = valid;
    }

    public XeroPoseEstimate() {
        this(new Pose2d(), 0.0, 0, false);
    }

    public static XeroPoseEstimate of(PoseEstimate llPoseEstimate) {
        return llPoseEstimate != null ? new XeroPoseEstimate(llPoseEstimate.pose, llPoseEstimate.timestampSeconds, llPoseEstimate.tagCount, true) : new XeroPoseEstimate();
    }

    public static final XeroPoseEstimateStruct struct = new XeroPoseEstimateStruct();
}