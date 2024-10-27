package org.xero1425.subsystems.vision;

import java.util.function.Supplier;

import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;

public class AprilTagVisionIOSim extends AprilTagVisionIOPhoton {

    private final VisionSystemSim sim_;
    private final PhotonCameraSim camSim_;
    private final SimCameraProperties camProps_;

    private final Supplier<Pose2d> robotPoseSupplier_;

    public AprilTagVisionIOSim(String name, Supplier<Pose2d> robotPoseSupplier, AprilTagFieldLayout layout, Transform3d robotToCamera, SimCameraProperties camProps) {
        super(name, layout, robotToCamera);
        
        // Create vision sim
        sim_ = new VisionSystemSim(name);
        
        // Setup apriltags in sim
        sim_.addAprilTags(layout);
        
        // Properties for camera simulation
        camProps_ = camProps;
        
        // Setup camera sim
        camSim_ = new PhotonCameraSim(camera_, camProps_);
        camSim_.enableDrawWireframe(true);
        
        sim_.addCamera(camSim_, robotToCamera_);

        robotPoseSupplier_ = robotPoseSupplier;
    }

    @Override
    public void updateInputs(AprilTagVisionIOInputsAutoLogged inputs) {
        sim_.update(robotPoseSupplier_.get());
        super.updateInputs(inputs);
    }
    
}