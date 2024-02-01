package Team4450.Robot24.subsystems;

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
import edu.wpi.first.math.geometry.Rotation3d;
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

    private final AprilTagFields    fields = AprilTagFields.k2024Crescendo;
    private AprilTagFieldLayout     FIELD_LAYOUT;
    private PhotonPoseEstimator     poseEstimator;

	public PhotonVision() 
	{
        FIELD_LAYOUT = fields.loadAprilTagLayoutField();

        // setup the AprilTag pose etimator
        poseEstimator = new PhotonPoseEstimator(
            FIELD_LAYOUT,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, // strategy to use for tag to pose calculation
            camera, // the PhotonCamera
            new Transform3d() // a series of transformations from Camera pos. to robot pos. (where camera is on robot)
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
        return getYaw(latestResult.getBestTarget().getFiducialId());
    }
    /**
     * 
     * @param fiducialID
     * @return yaw of target (tag) with fiducialID if valid and seen, area of target (other) if invalid and seen,
     *         else -1
     */
    public double getYaw(int fiducialID) {
        boolean validFiducialID = isFiducialIDValid(fiducialID);

        if (validFiducialID && hasTarget(fiducialID))
            return getTarget(fiducialID).getYaw();
        else if (!validFiducialID && hasTargets())
        {
            PhotonTrackedTarget target = getBestTarget(false).orElse(null);
            if (target != null) return target.getYaw();
        }
        
        return 0;
    }

    /**
     * Returns the area of the best target in the latest camera results
     * list. Must call hasTargets() before calling this function.
     * @return Best target area value.
     */
    public double getArea()
    {
        return getArea(latestResult.getBestTarget().getFiducialId());
    }
    /**
     * 
     * @param fiducialID
     * @return area of target (tag) with fiducialID if valid and seen, area of target (other) if invalid and seen,
     *         else -1
     */
    public double getArea(int fiducialID) {
        boolean validFiducialID = isFiducialIDValid(fiducialID);

        if (validFiducialID && hasTarget(fiducialID))
            return getTarget(fiducialID).getArea();
        else if (!validFiducialID && hasTargets())
        {
            PhotonTrackedTarget target = getBestTarget(false).orElse(null);
            if (target != null) return target.getArea();
        }
        
        return 0;
    }

    public Optional<PhotonTrackedTarget> getBestTarget(boolean hasID) {
        if (hasTargets())
            for (int bestTargetIndex = 0; bestTargetIndex < latestResult.getTargets().size(); bestTargetIndex++) {
                int targetID = latestResult.getTargets().get(bestTargetIndex).getFiducialId();
                if (isFiducialIDValid(targetID) == hasID)
                    return Optional.of(latestResult.getTargets().get(bestTargetIndex));
            }

        return Optional.empty();
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
     * Checks whether the id is within the valid range
     * @param id the id you are testing
     * @return whether or not the id is within the valid range
     */
    public boolean isFiducialIDValid(int id) {
        return id >= 0 && id <= 16;
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
    
    /**
     * returns an Optional value of the robot's estimated 
     * field-centric pose given current tags that it sees.
     * (and also the timestamp)
     * 
     * @return the Optional estimated pose (empty optional means no pose or uncertain/bad pose)
     */
    public Optional<EstimatedRobotPose> getEstimatedPose() {
        Optional<EstimatedRobotPose> estimatedPoseOptional = poseEstimator.update();
        
        if (estimatedPoseOptional.isPresent()) {
            EstimatedRobotPose estimatedPose = estimatedPoseOptional.get();
            Pose3d pose = estimatedPose.estimatedPose;

            // pose2d to pose3d (ignore the Z axis which is height off ground)
            Pose2d pose2d = new Pose2d(pose.getX(), pose.getY(), new Rotation2d(pose.getRotation().getAngle()));

            // update the field2d object in NetworkTables to visualize where the camera thinks it's at
            field.setRobotPose(pose2d);

            // logic for checking if pose is valid would go here:
            // for example:
            for (int i=0;i<estimatedPose.targetsUsed.size();i++) {
                // if a target was used with ID > 16 then return no estimated pose
                if (estimatedPose.targetsUsed.get(i).getFiducialId() > 16) {
                    return Optional.empty();
                }
            }

            return Optional.of(estimatedPose);
        } else return Optional.empty();
    }
    /**
     * @return pose of best april tag target, if no april tag target exists then instead return empty
     */
    public Optional<Pose3d> getTagPose() {
        return poseEstimator.getFieldTags().getTagPose(getBestTarget(true).get().getFiducialId());
    }
    /**
     * @return (if getEstimatedPose() && getTagPose() are not empty) transform from robot to best tag
    *       <p>(else) Optional.empty();
     */
    public Optional<Transform3d> getRobotToTag() {
        Optional<EstimatedRobotPose> optionalWorldPose = getEstimatedPose();
        Optional<Pose3d> optionalTagPose = getTagPose();
        if (optionalWorldPose.isEmpty() || optionalTagPose.isEmpty()) return Optional.empty();

        Pose3d worldPose = optionalWorldPose.get().estimatedPose;
        Pose3d tagPose = optionalTagPose.get();

        Transform3d tagRelevantTransform = worldPose.minus(tagPose);
        return Optional.of(tagRelevantTransform);
    }
    /**
     * 
     * @return
     */
    public Optional<Pose3d> getNotePose() {
        Optional<EstimatedRobotPose> optionalRobotWorldPose = getEstimatedPose();
        Optional<Transform3d> optionalNotePose = getRobotToNote();
        if (optionalRobotWorldPose.isEmpty() || optionalNotePose.isEmpty()) return Optional.empty();

        Pose3d robotWorldPose = optionalRobotWorldPose.get().estimatedPose;
        Transform3d robotToNote = optionalNotePose.get();
        
        //TODO: convert minus to plus
        Transform3d noteTransform = robotWorldPose.minus(new Pose3d(robotToNote.getTranslation(),
                                                                       robotToNote.getRotation()));
        Pose3d notePose = new Pose3d(noteTransform.getTranslation(), noteTransform.getRotation());
        return Optional.of(notePose);
    }
    /**
     * @return
     */
    public Optional<Transform3d> getRobotToNote() {
        if (hasTargets()) {
            EstimatedRobotPose robotWorldPose = getEstimatedPose().orElse(null);
            if (robotWorldPose == null) return Optional.empty();

            double bestTargetArea = getArea();
            // Unsure if objects size may get distorted when not looking directly at them, assuming they do not
            // and only distory base on size. If it ends up being true, will adjust once known how.
            double yaw = getYaw();

            final double WIDTH_OF_NOTE_MM = 14 * 25.4; // inch to millimeter
            final double CAM_FOCAL_LENGTH_MM = 50 * 10; // centimeter to millimeter
            final int CAM_TOTAL_PIXELS = 320 * 240;
            double pixelsTakenByNote = bestTargetArea * CAM_TOTAL_PIXELS;
            double robotToNoteMagnitude = (WIDTH_OF_NOTE_MM * CAM_FOCAL_LENGTH_MM) / pixelsTakenByNote;

            // gets a slight translation behind the robot (assuming .div works as I hope)
            Transform3d baseTransform = new Transform3d(robotWorldPose.estimatedPose, 
                                                        robotWorldPose.estimatedPose.div(-0.1));

            double baseTransformMagnitude = Math.sqrt(Math.pow(baseTransform.getX(), 2)
                                             + Math.pow(baseTransform.getY(), 2));
            double magnitudeMultiplier = 1 / baseTransformMagnitude * robotToNoteMagnitude;
            
            double cosRadians = Math.cos(yaw);
            double sinRadians = Math.sin(yaw);
            double adjustedX = baseTransform.getX() * cosRadians
                             - baseTransform.getY() * sinRadians
                             * magnitudeMultiplier;
            double adjustedY = baseTransform.getX() * sinRadians
                             + baseTransform.getY() * cosRadians
                             * magnitudeMultiplier;
            Transform3d robotToNoteTransform = new Transform3d(adjustedX,
                                                               adjustedY,
                                                               0,
                                                               new Rotation3d(0,
                                                                              0,
                                                                              yaw));

            return Optional.of(robotToNoteTransform);
        }
        else
            return Optional.empty();
    }
    /**
     * converts poseEstimator.getReferencePose() to a Optional<Pose3d> so it can return empty if unknown
     */
    public Optional<Pose3d> getReferencePose() {
        Pose3d referencePose = poseEstimator.getReferencePose();

        return referencePose != null ? Optional.of(referencePose) : Optional.empty();
    }
    public Optional<Transform3d> robotToTarget() {
        if (hasTargets())
            return Optional.of(latestResult.getBestTarget().getBestCameraToTarget().plus(new Transform3d().inverse()));
        else
            return Optional.empty();
    }
}