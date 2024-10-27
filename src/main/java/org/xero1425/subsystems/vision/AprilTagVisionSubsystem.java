package org.xero1425.subsystems.vision;

import java.util.ArrayList;
import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.xero1425.struct.vision.XeroFiducial;
import org.xero1425.struct.vision.XeroPoseEstimate;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AprilTagVisionSubsystem extends SubsystemBase {

    private final AprilTagVisionIO io_;
    private final AprilTagVisionIOInputsAutoLogged inputs_;
    
    private final AprilTagFieldLayout layout_;

    private Supplier<Pose2d> poseSupplier_;
    private Consumer<XeroPoseEstimate> megatagConsumer_;

    @AutoLogOutput(key = "AprilTagVision/LastValidTargetTimestamp")
    private double lastValidTargetTimestamp_;

    /**
     * Creates a vision subsystem.
     * @param io The IO implementation to use
     * @param layout The current field layout for the game
     * @param poseSupplier A supplier that gets the current estimated robot pose
     * @param megatagConsumer A consumer which takes a new estimated pose and fuses it into pose.
     */
    public AprilTagVisionSubsystem(AprilTagVisionIO io, AprilTagFieldLayout layout, Supplier<Pose2d> poseSupplier, Consumer<XeroPoseEstimate> megatagConsumer) {
        io_ = io;
        inputs_ = new AprilTagVisionIOInputsAutoLogged();
        layout_ = layout;

        poseSupplier_ = poseSupplier;
        megatagConsumer_ = megatagConsumer;
        lastValidTargetTimestamp_ = 0;
    }

    @Override
    public void periodic() {
        io_.updateInputs(inputs_);
        Logger.processInputs(getName(), inputs_);

        // give the camera an orientation
        io_.giveRobotOrientation(poseSupplier_.get().getRotation().getDegrees(), 0, 0, 0, 0, 0);
                
        // give the megatag2 estimate to consumers (ex. drivebase vision measurement)
        megatagConsumer_.accept(inputs_.poseEstimate);

        if (hasValidTarget()) {
            lastValidTargetTimestamp_ = Timer.getFPGATimestamp();
        }

        // Creates an array of poses of the currently visible tags. This is useful for seeing which tags the robot can see visually.
        ArrayList<Pose3d> validTargetPoses = new ArrayList<Pose3d>();

        for (XeroFiducial fiducial : inputs_.fiducials) {
            layout_.getTagPose((int) fiducial.id).ifPresent((pose) -> {
                validTargetPoses.add(pose);
            });
        }

        Logger.recordOutput(getName() + "/ValidTargetPoses", validTargetPoses.toArray(new Pose3d[0]));
    }

    /**
     * Forces the camera indicator LED to be off.
     */
    public Command forceLedOff() {
        return runOnce(io_::forceOff);
    }

    /**
     * Forces the camera indicator LED to blink.
     */
    public Command forceLedBlink() {
        return runOnce(io_::forceBlink);
    }

    /**
     * Forces the camera indicator LED to be on.
     */
    public Command forceLedOn() {
        return runOnce(io_::forceOn);
    }

    /**
     * Sets the camera indicator LED back to the default state, controlled by the camera software itself.
     */
    public Command resetLed() {
        return runOnce(io_::resetLed);
    }

    /**
     * Gets estimated information from the regular pose estimation.
     * @return Estimated pose.
     */
    public XeroPoseEstimate getPoseEstimate() {
        return inputs_.poseEstimate;
    }

    /**
     * Figures out if its currently seeing a specific april tag.
     * @param id The id of the april tag.
     * @return Whether or not the camera can currently see the specified april tag.
     */
    public boolean hasAprilTag(int id) {
        return findFid(id).isPresent();
    }

    /**
     * Finds if a valid target exists.
     * @return Whether or not a valid target exists.
     */
    public boolean hasValidTarget() {
        return inputs_.simpleValid;
    }

    /**
     * Gets the id of the primary in-view Fiducial/Apriltag
     * @return
     */
    public int getSimpleID() {
        return inputs_.simpleID;
    }

    /**
     * Gets the vision targets X offset. Make sure to check if the robot is seeing a tag, and its the one you want first. Otherwise this data will be innaccurate.
     * @return Its X offset in degrees from the center of the camera. +X Right +Y Down
     */
    public double getSimpleX() {
        return inputs_.simpleX;
    }

    /**
     * Gets the vision targets Y offset. Make sure to check if the robot is seeing a tag, and its the one you want first. Otherwise this data will be innaccurate.
     * @return Its Y offset in degrees from the center of the camera. +X Right +Y Down
     */
    public double getSimpleY() {
        return inputs_.simpleY;
    }

    /**
     * Gets the vision targets area on the camera. Make sure to check if the robot is seeing a tag, and its the one you want first. Otherwise this data will be innaccurate.
     * @return How much of the camera the target covers. This range is configured in the camera tuning.
     */
    public double getSimpleArea() {
        return inputs_.simpleArea;
    }

    /**
     * Gets a specific apriltag's X offset. Make sure to check if the robot can see the tag first. Otherwise this data will be innaccurate.
     * @param id The id of the apriltag.
     * @return Its X offset in degrees from the center of the camera. +X Right +Y Down
     */
    public Optional<Double> getSpecificX(int id) {
        Optional<XeroFiducial> fid = findFid(id);

        if (fid.isEmpty())
            return Optional.empty();

        return Optional.of(fid.get().x);
    }

    /**
     * Gets a specific apriltag's Y offset. Make sure to check if the robot can see the tag first. Otherwise this data will be innaccurate.
     * @param id The id of the apriltag.
     * @return Its Y offset in degrees from the center of the camera. +X Right +Y Down
     */
    public Optional<Double> getSpecificY(int id) {
        Optional<XeroFiducial> fid = findFid(id);

        if (fid.isEmpty())
            return Optional.empty();

        return Optional.of(fid.get().y);
    }

    /**
     * Gets a specific apriltag's area on the camera. Make sure to check if the robot can see the tag first. Otherwise this data will be innaccurate.
     * @param id The id of the apriltag.
     * @return How much of the camera the tag covers. This range is configured in the camera tuning.
     */
    public Optional<Double> getSpecificArea(int id) {
        Optional<XeroFiducial> fid = findFid(id);

        if (fid.isEmpty())
            return Optional.empty();

        return Optional.of(fid.get().area);
    }

    /**
     * Gets the time since theres been a valid target.
     * @return The time elapsed, in seconds.
     */
    public double getTimeSinceValidTarget() {
        return Timer.getFPGATimestamp() - lastValidTargetTimestamp_;
    }

    /**
     * Finds a fidicial object in the array.
     * @return The fidicial, null if not found.
     */
    private Optional<XeroFiducial> findFid(int id) {
        for (XeroFiducial fid : inputs_.fiducials) {
            if (fid.id == (double) id) {
                return Optional.of(fid);
            }
        }

        return Optional.empty();
    }

}