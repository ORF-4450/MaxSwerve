package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import Team4450.Lib.Util;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhotonVision extends SubsystemBase
{
    private PhotonCamera            camera = new PhotonCamera("4450-LL");
    private PhotonPipelineResult    latestResult;
    private VisionLEDMode           ledMode = VisionLEDMode.kOff;

    private Field2d field = new Field2d();

    // adams code ==========
    private final AprilTagFields fields = AprilTagFields.k2024Crescendo;
    private AprilTagFieldLayout FIELD_LAYOUT;
    private PhotonPoseEstimator poseEstimator;

    // end adams code=============

	public PhotonVision() 
	{
        FIELD_LAYOUT = fields.loadAprilTagLayoutField();
        poseEstimator = new PhotonPoseEstimator(
            FIELD_LAYOUT,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            camera,
            new Transform3d()
        );
        setLedMode(ledMode);

		Util.consoleLog("PhotonVision created!");

        SmartDashboard.putData(field);
	}

    /**
     * Get the lastest target results object returned by the camera.
     * @return Results object.
     */
    public PhotonPipelineResult getLatestResult()
    {
        latestResult = camera.getLatestResult();

        return latestResult;
    }

    /**
     * Indicates if lastest camera results list contains targets. Must 
     * call getLatestResult() before calling.
     * @return True if targets available, false if not.
     */
    public boolean hasTargets()
    {
        getLatestResult();

        return latestResult.hasTargets();
    }

    /**
     * Returns the target with the given Fiducial ID
     * @param id the desired Fiducial ID
     * @return the target or null if the ID is not currently being tracked
     */
    public PhotonTrackedTarget getTarget(int id)
    {
        if (hasTargets()) {
            List<PhotonTrackedTarget> targets = latestResult.getTargets();
            for (int i=0;i<targets.size();i++) {
                PhotonTrackedTarget target = targets.get(i);
                if (target.getFiducialId() == id) return target;
            }
            return null;
        }
        else
            return null;
    }
    
    /**
     * Get an array of the currently tracked Fiducial IDs
     * 
     * @return an ArrayList of the tracked IDs
     */
    public ArrayList<Integer> getTrackedIDs() {
        ArrayList<Integer> ids = new ArrayList<Integer>();
        if (hasTargets()) {
            List<PhotonTrackedTarget> targets = latestResult.getTargets();
            for (int i=0;i<targets.size();i++) {
                ids.add(targets.get(i).getFiducialId());
            }
        }
        return ids;
    }

    /**
     * Checks whether or not the camera currently sees a target
     * with the given Fiducial ID
     * 
     * @param id the Fiducial ID
     * @return whether the camera sees the ID
     */
    public boolean hasTarget(int id) {
        return getTrackedIDs().contains(id);
    }

    // Best Target Methods =============================================================

    /**
     * Returns the yaw angle of the best target in the latest camera results
     * list. Must call hasTargets() before calling this function.
     * @return Best target yaw value from straight ahead or zero. -yaw means
     * target is left of robot center.
     */
    public double getYaw()
    {
        if (hasTargets()) 
            return latestResult.getBestTarget().getYaw();
        else
            return 0;
    }


    /**
     * Returns the Fiducial ID of the current best target, you should call
     * hasTargets() first!
     * @return the ID or -1 if no targets
     */
    public int getFiducialID()
    {
        if (hasTargets()) 
            return latestResult.getBestTarget().getFiducialId();
        else
            return -1;
    }

    /**
     * Returns the area of the best target in the latest camera results
     * list. Must call hasTargets() before calling this function.
     * @return Best target area value.
     */
    public double getArea()
    {
        if (hasTargets()) 
            return latestResult.getBestTarget().getArea();
        else
            return 0;
    }
 
    // Utility Methods =============================================================

    /**
     * Select camera's image processing pipeline.
     * @param index Zero based number of desired pipeline.
     */
    public void selectPipeline(int index)
    {
        Util.consoleLog("%d", index);

        camera.setPipelineIndex(index);
    }

    /**
     * Set the LED mode.
     * @param mode Desired LED mode.
     */
    public void setLedMode(VisionLEDMode mode)
    {
        Util.consoleLog("%d", mode.value);

        camera.setLED(mode);

        ledMode = mode;
    }

    /**
     * Toggle LED mode on/off.
     */
    public void toggleLedMode()
    {
        if (ledMode == VisionLEDMode.kOff)
            ledMode = VisionLEDMode.kOn;
        else
            ledMode = VisionLEDMode.kOff;
        
        setLedMode(ledMode);
    }

    /**
     * Save pre-processed image from camera stream.
     */
    public void inputSnapshot()
    {
        Util.consoleLog();

        camera.takeInputSnapshot();
    }

    /**
     * Save post-processed image from camera stream.
     */
    public void outputSnapshot()
    {
        Util.consoleLog();

        camera.takeOutputSnapshot();
    }
        
    @Override
	public void initSendable( SendableBuilder builder )
	{
        //super.initSendable(builder);
        builder.setSmartDashboardType("Subsystem");

        builder.addBooleanProperty("has Targets", () -> hasTargets(), null);
        builder.addDoubleProperty("target yaw", () -> getYaw(), null);
        builder.addDoubleProperty("target area", () -> getArea(), null);
	}
    
    // TODO: document
    public Optional<Pose2d> getEstimatedPose() {
        Optional<EstimatedRobotPose> estimatedPoseOptional = poseEstimator.update();
        if (estimatedPoseOptional.isPresent()) {
            EstimatedRobotPose estimatedPose = estimatedPoseOptional.get();
            Pose3d pose = estimatedPose.estimatedPose;
            Pose2d pose2d = new Pose2d(pose.getX(), pose.getY(), new Rotation2d(pose.getRotation().getAngle()));
            field.setRobotPose(pose2d);
            return Optional.of(pose2d);
        } else return Optional.empty();
    }
}