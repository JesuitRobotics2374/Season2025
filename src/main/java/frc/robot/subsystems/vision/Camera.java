package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;

public class Camera {
    private PhotonCamera camera;
    private cameraType type;
    private PhotonPoseEstimator poseEstimator;
    private Transform3d robotToCameraTransform;

    public Camera() {

    }

    /**
     * Adjust the raw pose obtained from the camera to account for the camera's position and orientation on the robot.
     * @param rawPose, the raw Pose3d obtained from the camera
     * @return Adjusted Pose3d representing the robot's pose on the field
     */
    private Pose3d adjustPose(Pose3d rawPose) {
        if (rawPose == null) {
            return null;
        }

        return rawPose.transformBy(robotToCameraTransform); //TODO: CHECK IF THIS NEEDS TO BE INVERSED OR SPLIT INTO COMPONENTS
    }

    /**
     * Get the global field pose of the robot as estimated by the camera.
     * @return EstimatedRobotPose object containing the robot's pose and targets used to find this, or null if no valid pose is available.
     */
    public EstimatedRobotPose getGlobalFieldPose() {

        // TODO: CHECK IF THIS NEEDS TO BE CHECKED FOR CAMRERATYPE

        List<PhotonPipelineResult> unreadResults = camera.getAllUnreadResults(); // Get all unread results from the camera
        if (unreadResults.isEmpty()) { // If there are no unread results, return null
            return null;
        }
        PhotonPipelineResult latestResult = unreadResults.get(unreadResults.size() - 1); // Get the latest result
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
    public ArrayList<Integer> allAvailableTags() {

        // TODO: CHECK IF THIS NEEDS TO BE CHECKED FOR CAMRERATYPE

        ArrayList<Integer> tagIDs = new ArrayList<Integer>(); // Initialize an empty list to store tag IDs

        List<PhotonPipelineResult> unreadResults = camera.getAllUnreadResults(); // Get all unread results from the camera
        if (unreadResults.isEmpty()) { // If there are no unread results, return the empty list
            return tagIDs;
        }
        PhotonPipelineResult latestResult = unreadResults.get(unreadResults.size() - 1); // Get the latest result
        if (latestResult == null || !latestResult.hasTargets()) { // If there are no targets in the latest result, return the empty list
            return tagIDs;
        }

        for (PhotonTrackedTarget target : latestResult.getTargets()) { // Iterate through each detected target
            tagIDs.add(target.getFiducialId()); // Add the tag ID to the list
        }

        return tagIDs; // Return the list of tag IDs
    }




} 
