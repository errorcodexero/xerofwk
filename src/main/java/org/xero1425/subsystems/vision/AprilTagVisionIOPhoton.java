package org.xero1425.subsystems.vision;

import java.util.ArrayList;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;
import org.xero1425.struct.vision.XeroFiducial;
import org.xero1425.struct.vision.XeroPoseEstimate;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;

public class AprilTagVisionIOPhoton implements AprilTagVisionIO {

    // Transform from robot to camera.
    protected final Transform3d robotToCamera_;

    protected final PhotonCamera camera_;
    private final PhotonPoseEstimator poseEstimator_;

    public AprilTagVisionIOPhoton(String name, AprilTagFieldLayout layout, Transform3d robotToCamera) {
        // Setup camera
        camera_ = new PhotonCamera(name);

        robotToCamera_ = robotToCamera;

        // Setup pose estimator
        poseEstimator_ = new PhotonPoseEstimator(
            layout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            camera_,
            robotToCamera_
        );
    }

    @Override
    public void updateInputs(AprilTagVisionIOInputsAutoLogged inputs) {
        PhotonPipelineResult result = camera_.getLatestResult();
        PhotonTrackedTarget bestTarget = result.getBestTarget();

        // Simple setup
        if (bestTarget != null) {
            inputs.simpleID = bestTarget.getFiducialId();
            inputs.simpleX = bestTarget.getPitch();
            inputs.simpleY = bestTarget.getYaw();
            inputs.simpleArea = bestTarget.getArea();
            inputs.simpleValid = true;
        } else {
            inputs.simpleID = 0;
            inputs.simpleX = 0.0;
            inputs.simpleY = 0.0;
            inputs.simpleArea = 0.0;
            inputs.simpleValid = false;
        }

        // Target information to fill
        ArrayList<Translation2d> cornerCoords = new ArrayList<>();
        ArrayList<XeroFiducial> fiducials = new ArrayList<>();

        // Get target information
        for (PhotonTrackedTarget target : result.getTargets()) {

            for (TargetCorner corner : target.getDetectedCorners()) {
                cornerCoords.add(new Translation2d(corner.x, corner.y));
            }

            fiducials.add(new XeroFiducial(
                target.getFiducialId(),
                target.getArea(),
                target.getPitch(),
                target.getYaw()
            ));
        }

        inputs.rawCorners = cornerCoords.toArray(new Translation2d[0]);
        inputs.fiducials = fiducials.toArray(new XeroFiducial[0]);

        Optional<EstimatedRobotPose> optionalPhotonEstimate = poseEstimator_.update();

        optionalPhotonEstimate.ifPresentOrElse(photonEstimate -> {
            inputs.poseEstimate = new XeroPoseEstimate(
                photonEstimate.estimatedPose.toPose2d(),
                photonEstimate.timestampSeconds,
                photonEstimate.targetsUsed.size(),
                true
            );
        }, () -> {
            inputs.poseEstimate = new XeroPoseEstimate();
        });
    }

    @Override
    public void forceBlink() {
        camera_.setLED(VisionLEDMode.kBlink);
    }

    @Override
    public void forceOff() {
        camera_.setLED(VisionLEDMode.kOff);
    }

    @Override
    public void forceOn() {
        camera_.setLED(VisionLEDMode.kOn);
    }

    @Override
    public void resetLed() {
        camera_.setLED(VisionLEDMode.kDefault);
    }

}