package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Core;

public class Camera {
    private PhotonCamera camera; // The PhotonCamera instance
    private Type type; // The type of object the camera detects
    private PhotonPoseEstimator poseEstimator; // The pose estimator for AprilTag cameras
    private Transform3d robotToCameraTransform; // The transform from the robot to the camera
    private PhotonPipelineResult latestResult; // The latest result from the camera

    // Enum for the type of object the camera detects
    public enum Type {
        APRIL_TAG, 
        CORAL, 
        ALGAE
    }

    /**
     * Constructor for the Camera class.
     */
    public Camera(NetworkTableInstance nti, String cameraName, Transform3d robotToCameraTransform) {
        this.camera = new PhotonCamera(nti, cameraName);
        this.robotToCameraTransform = robotToCameraTransform;

        Type[] values = Type.values();
        this.type = values[camera.getPipelineIndex()];

        if (this.type == Type.APRIL_TAG) {
            AprilTagFieldLayout aprilTagFieldLayout = null;

            poseEstimator = new PhotonPoseEstimator(
                aprilTagFieldLayout, 
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, 
                robotToCameraTransform
            );

            poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        }
        else {
            poseEstimator = null;
        }
    }


    /**
     * Updates the camera to store its latest results for use in methods. 
     */
    public void updateResults() {
        List<PhotonPipelineResult> unreadResults = camera.getAllUnreadResults(); // Get all unread results from the camera

        if (unreadResults.size() == 0) { // If there are no unread results, ignore changing anything
            return;
        }
        else {
            latestResult = unreadResults.get(unreadResults.size() - 1); // Update latestResult to the most recent unread result
        }
    }

    /**
     * Adjust the raw pose obtained from the camera to account for the camera's position and orientation on the robot.
     * @param rawPose, the raw Pose3d obtained from the camera.
     * @return Adjusted Pose3d representing the robot's pose on the field.
     */
    private Pose3d adjustPose(Transform3d rawPose) {
        if (rawPose == null) { // If the raw pose is null, return null
            return null;
        }

        Transform3d adjustedPose = rawPose.plus(robotToCameraTransform); // Adjust the raw pose by adding the robot-to-camera transform, TODO: CHECK IF THIS NEEDS TO BE SUBTRACTED

        return new Pose3d(adjustedPose.getTranslation(), adjustedPose.getRotation()); // Return the adjusted pose as a Pose3d
    }

    /**
     * Get the global field pose of the robot as estimated by the camera.
     * @return EstimatedRobotPose object containing the robot's pose and targets used to find this, or null if no valid pose is available.
     */
    public EstimatedRobotPose getGlobalFieldPose() {
        if (!type.equals(Type.APRIL_TAG)) { // If the type is not for an AprilTag, return null
            return null;
        }

        if (latestResult == null || !latestResult.hasTargets()) { // If there are no targets in the latest result, return null
            return null;
        }

        Optional<EstimatedRobotPose> estimatedRobotPose = poseEstimator.update(latestResult); // Estimate the robot's pose using the latest result

        if (estimatedRobotPose.isPresent()) { // If a valid pose is estimated, return it
            return estimatedRobotPose.get();
        }

        return null; // Return null if no valid pose is estimated
    }

    /**
     * Get a list of all available AprilTag IDs detected by the camera.
     * @return ArrayList of Integer tag IDs.
     */
    public ArrayList<Integer> getAllAvailableTags() {
        if (!type.equals(Type.APRIL_TAG)) { // If the type is not for an AprilTag, return null
            return null;
        }

        ArrayList<Integer> tagIDs = new ArrayList<Integer>(); // Initialize an empty list to store tag IDs

        if (latestResult == null || !latestResult.hasTargets()) { // If there are no targets in the latest result, return null
            return tagIDs;
        }

        for (PhotonTrackedTarget target : latestResult.getTargets()) { // Iterate through each detected target
            tagIDs.add(target.getFiducialId()); // Add the tag ID to the list
        }

        return tagIDs; // Return the list of tag IDs
    }

    /**
     * Get the pose of a specific AprilTag relative to the robot.
     * @param tagID, the ID of the AprilTag to find.
     * @return Transform3d of the specified tag relative to the robot, or null if not found.
     */
    public Pose3d getTagRelativeToBot(int tagID) {
        if (!type.equals(Type.APRIL_TAG)) { // If the type is not for an AprilTag, return null
            return null;
        }

        if (latestResult == null || !latestResult.hasTargets()) { // If there are no targets in the latest result, return null
            return null;
        }

        for (PhotonTrackedTarget target : latestResult.getTargets()) { // Iterate through each detected target
            if (target.getFiducialId() == tagID) { // If the target's ID matches the requested tagID
                Transform3d cameraToTarget = target.getBestCameraToTarget(); // Get the transform from the camera to the target
                
                return adjustPose(cameraToTarget); // Return the pose of the tag relative to the robot
            }
        }

        return null; // Return null if the specified tagID is not found
    }

    /**
     * Get the pose of a specific AprilTag relative to the robot.
     * @param tagID, the ID of the AprilTag to find.
     * @return Pose3d of the specified tag relative to the robot, or null if not found.
     */
    public Pose3d getBotRelativeToTag(int tagID) {
        Pose3d tagRelativeToBot = getTagRelativeToBot(tagID); // Get the pose of the tag relative to the robot

        if (tagRelativeToBot == null) { // If the tag relative to bot is null, return null
            return null;
        }

        Transform3d botRelativeToTag = new Transform3d(tagRelativeToBot.getTranslation(), tagRelativeToBot.getRotation()).inverse(); // Invert the transform to get the robot relative to the tag

        return new Pose3d(botRelativeToTag.getTranslation(), botRelativeToTag.getRotation()); // Return the pose of the robot relative to the tag
    }

    /**
     * Get the type of object the camera detects.
     * @return Type enum representing the camera type.
     */
    public Type getCameraType() {
        return type;
    }

    /**
     * Get the current primary pose strategy used by the pose estimator.
     * @return PoseStrategy representing the current primary pose strategy, or null if not an AprilTag camera.
     */
    public PoseStrategy getPrimaryPoseStrategy() {
        if (!type.equals(Type.APRIL_TAG)) { // If the type is not for an AprilTag, return null
            return null;
        }

        return poseEstimator.getPrimaryStrategy();
    }

    /**
     * Change the primary pose strategy used by the pose estimator.
     * @param newStrategy, the new PoseStrategy to set as primary.
     */
    public void changePrimaryPoseStrategy(PoseStrategy newStrategy) {
        if (!type.equals(Type.APRIL_TAG)) { // If the type is not for an AprilTag, do nothing
            return;
        }

        poseEstimator.setPrimaryStrategy(newStrategy); // Change the primary pose strategy of the pose estimator
    }
}
