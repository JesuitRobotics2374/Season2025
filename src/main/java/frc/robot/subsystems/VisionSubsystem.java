package frc.robot.subsystems;

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
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;

public class VisionSubsystem {

    private static int numberOfCams = Constants.numberOfCams;
    private static PhotonCamera[] cameras = new PhotonCamera[numberOfCams];
    private static PhotonPoseEstimator[] poseEstimators = new PhotonPoseEstimator[numberOfCams];
    private static Transform3d[] cameraToBotRelativePose = {
            new Transform3d(0.176, 0.223, 0.255, new Rotation3d(0, 0, Math.toRadians(-7))) // x and y may be switched
    };
    private static AprilTagFieldLayout fieldLayout;

    /**
     * initializes the vision subsystem with the proper PhotonCameras and
     * photonPoseEstimators
     */
    public static void initializeVisionSubsystem() {
        System.out.println(NetworkTableInstance.getDefault());
        loadField();

        for (int i = 0; i < numberOfCams; i++) {
            cameras[i] = new PhotonCamera(NetworkTableInstance.getDefault(), "camera" + i);
            poseEstimators[i] = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                    cameraToBotRelativePose[i]);
            poseEstimators[i].setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        }
    }

    /**
     * loads the field
     */
    private static void loadField() {
        fieldLayout = AprilTagFieldLayout
                .loadField(AprilTagFields.k2025ReefscapeWelded);
    }

    /**
     * 
     * @return the average distance to the apriltag (chosen by PhotonVision) of all
     *         the cameras
     */
    public static double getDistanceToAprilTag() {
        double totalDistance = 0;
        int count = 0;

        for (int i = 0; i < numberOfCams; i++) {
            double distance = getDistanceToAprilTag(cameras[i]);
            if (distance != -1) {
                totalDistance += distance;
                count++;
            }
        }

        if (count == 0) {
            return -1;
        }

        return totalDistance / count;
    }

    /**
     * 
     * @param camera, a PhotonVision camera
     * @return the distance to the apriltag (chosen by PhotonVision)
     */
    private static double getDistanceToAprilTag(PhotonCamera camera) {
        PhotonPipelineResult result = camera.getLatestResult();

        if (result != null && result.hasTargets()) {

            List<PhotonTrackedTarget> targets = result.targets;

            if (targets.size() == 0) {
                return -1;
            }

            PhotonTrackedTarget target = result.getBestTarget();
            Transform3d transform = target.getBestCameraToTarget();

            return transform.getTranslation().getNorm();
        }

        return -1;
    }

    /**
     * 
     * @param priorityTagID, the priority tag ID at which to specifically look for
     * @return the average distance to the specified apriltag for all of the cameras
     */
    public static double getDistanceToAprilTag(int priorityTagID) {
        double totalDistance = 0;
        int count = 0;

        for (int i = 0; i < numberOfCams; i++) {
            double distance = getDistanceToAprilTag(cameras[i], priorityTagID);
            if (distance != -1) {
                totalDistance += distance;
                count++;
            }
        }

        if (count == 0) {
            return -1;
        }

        return totalDistance / count;
    }

    /**
     * 
     * @param camera,        a PhotonVision camera
     * @param priorityTagID, the priority tag ID at which to specifically look for
     * @return the distance to the specified apriltag
     */
    private static double getDistanceToAprilTag(PhotonCamera camera, int priorityTagID) {
        PhotonPipelineResult result = camera.getLatestResult();

        if (result != null && result.hasTargets()) {

            List<PhotonTrackedTarget> targets = result.targets;

            if (targets.size() == 0) {
                return -1;
            }

            for (PhotonTrackedTarget target : targets) {
                if (target.getFiducialId() == priorityTagID) {
                    return target.getBestCameraToTarget().getTranslation().getNorm();
                }
            }
        }

        return -1;
    }

    /**
     * 
     * @return the average Pose3d of the apriltag (chosen by PhotonVision) relative
     *         to the bot for all of the cameras
     */
    public static Pose3d getTagRelativeToBot() {
        double x = 0;
        double y = 0;
        double z = 0;
        double roll = 0;
        double pitch = 0;
        double yaw = 0;
        int count = 0;

        for (int i = 0; i < numberOfCams; i++) {
            Pose3d pose = getTagRelativeToBot(cameras[i]);

            if (pose != null) {
                x += pose.getX();
                y += pose.getY();
                z += pose.getZ();

                roll += pose.getRotation().getX();
                pitch += pose.getRotation().getY();
                yaw += pose.getRotation().getZ();
                count++;
            }
        }

        if (count == 0) {
            return null;
        }

        return new Pose3d(x / count, y / count, z / count, new Rotation3d(roll / count, pitch / count, yaw / count));
    }

    /**
     * 
     * @param camera, a PhotonVision camera
     * @return the Pose3d of the apriltag (chosen by PhotonVision) relative to the
     *         bot
     */
    private static Pose3d getTagRelativeToBot(PhotonCamera camera) {
        if (!canSeeTag(camera)) {
            return null;
        }

        PhotonPipelineResult result = camera.getLatestResult();

        if (result != null && result.hasTargets()) {

            List<PhotonTrackedTarget> targets = result.targets;

            if (targets.size() == 0) {
                return null;
            }

            Transform3d transform3d = result.getBestTarget().bestCameraToTarget;

            Translation3d translation3d = transform3d.getTranslation();
            Rotation3d rotation3d = transform3d.getRotation();

            return new Pose3d(translation3d, rotation3d);
        }

        return null;
    }

    /**
     * 
     * @param priorityTagID, the priority tag ID at which to specifically look for
     * @return the average Pose3d of the specified apriltag relative to the bot for
     *         all of the cameras
     */
    public static Pose3d getTagRelativeToBot(int priorityTagID) {
        double x = 0;
        double y = 0;
        double z = 0;
        double roll = 0;
        double pitch = 0;
        double yaw = 0;
        int count = 0;

        for (int i = 0; i < numberOfCams; i++) {
            Pose3d pose = getTagRelativeToBot(cameras[i], priorityTagID);

            if (pose != null) {
                x += pose.getX();
                y += pose.getY();
                z += pose.getZ();

                roll += pose.getRotation().getX();
                pitch += pose.getRotation().getY();
                yaw += pose.getRotation().getZ();
                count++;
            }
        }

        if (count == 0) {
            return null;
        }

        return new Pose3d(x / count, y / count, z / count, new Rotation3d(roll / count, pitch / count, yaw / count));
    }

    /**
     * 
     * @param camera,        a PhotonVision camera
     * @param priorityTagID, the priority tag ID at which to specifically look for
     * @return the Pose3d of the specified apriltag relative to the bot
     */
    private static Pose3d getTagRelativeToBot(PhotonCamera camera, int priorityTagID) {
        if (!canSeeTag(camera)) {
            return null;
        }

        PhotonPipelineResult result = camera.getLatestResult();

        if (result != null && result.hasTargets()) {

            List<PhotonTrackedTarget> targets = result.targets;

            if (targets.size() == 0) {
                return null;
            }

            for (PhotonTrackedTarget target : targets) {
                if (target.getFiducialId() == priorityTagID) {
                    Transform3d transform3d = target.getBestCameraToTarget();

                    Translation3d translation3d = transform3d.getTranslation();
                    Rotation3d rotation3d = transform3d.getRotation();

                    return new Pose3d(translation3d, rotation3d);
                }
            }
        }

        return null;
    }

    /**
     * 
     * @return the average Pose3d of the bot relative to the apriltag (chosen by
     *         PhotonVision) for all of the cameras
     */
    public static Pose3d getBotRelativeToTag() {
        double x = 0;
        double y = 0;
        double z = 0;
        double roll = 0;
        double pitch = 0;
        double yaw = 0;
        int count = 0;

        for (int i = 0; i < numberOfCams; i++) {
            Pose3d pose = getBotRelativeToTag(cameras[i]);

            if (pose != null) {
                x += pose.getX();
                y += pose.getY();
                z += pose.getZ();

                roll += pose.getRotation().getX();
                pitch += pose.getRotation().getY();
                yaw += pose.getRotation().getZ();
                count++;
            }
        }

        if (count == 0) {
            return null;
        }

        return new Pose3d(x / count, y / count, z / count, new Rotation3d(roll / count, pitch / count, yaw / count));
    }

    /**
     * 
     * @param camera, a PhotonVision camera
     * @return the Pose3d of the bot relative to the apriltag (chosen by
     *         PhotonVision)
     */
    private static Pose3d getBotRelativeToTag(PhotonCamera camera) {
        if (!canSeeTag(camera)) {
            return null;
        }

        PhotonPipelineResult result = camera.getLatestResult();

        if (result != null && result.hasTargets()) {

            List<PhotonTrackedTarget> targets = result.targets;

            if (targets.size() == 0) {
                return null;
            }

            Transform3d transform3d = result.getBestTarget().bestCameraToTarget;

            Translation3d translation3d = transform3d.getTranslation();
            Rotation3d rotation3d = transform3d.getRotation();

            return new Pose3d(new Translation3d(-translation3d.getX(), -translation3d.getY(), -translation3d.getZ()),
                    rotation3d);
        } else {
            return null;
        }
    }

    /**
     * 
     * @param priorityTagID, the priority tag ID at which to specifically look for
     * @return the average Pose3d of the bot relative to the specified apriltag for
     *         all of the cameras
     */
    public static Pose3d getBotRelativeToTag(int priorityTagID) {
        double x = 0;
        double y = 0;
        double z = 0;
        double roll = 0;
        double pitch = 0;
        double yaw = 0;
        int count = 0;

        for (int i = 0; i < numberOfCams; i++) {
            Pose3d pose = getBotRelativeToTag(cameras[i], priorityTagID);

            if (pose != null) {
                x += pose.getX();
                y += pose.getY();
                z += pose.getZ();

                roll += pose.getRotation().getX();
                pitch += pose.getRotation().getY();
                yaw += pose.getRotation().getZ();
                count++;
            }
        }

        if (count == 0) {
            return null;
        }

        return new Pose3d(x / count, y / count, z / count, new Rotation3d(roll / count, pitch / count, yaw / count));
    }

    /**
     * 
     * @param camera,        a PhotonVision camera
     * @param priorityTagID, the priority tag ID at which to specifically look for
     * @return the Pose3d of the bot relative to the specified apriltag
     */
    private static Pose3d getBotRelativeToTag(PhotonCamera camera, int priorityTagID) {
        if (!canSeeTag(camera)) {
            return null;
        }

        PhotonPipelineResult result = camera.getLatestResult();

        if (result != null && result.hasTargets()) {

            List<PhotonTrackedTarget> targets = result.targets;

            if (targets.size() == 0) {
                return null;
            }

            for (PhotonTrackedTarget target : targets) {
                if (target.getFiducialId() == priorityTagID) {
                    Transform3d transform3d = target.getBestCameraToTarget();

                    Translation3d translation3d = transform3d.getTranslation();
                    Rotation3d rotation3d = transform3d.getRotation();

                    return new Pose3d(translation3d, rotation3d);
                }
            }
        }

        return null;
    }

    /**
     * 
     * @param camera, a PhotonVision camera
     * @return if the camera can see ANY tag
     */
    private static boolean canSeeTag(PhotonCamera camera) {
        return camera.getLatestResult().hasTargets();
    }

    /**
     * 
     * @return A list of all EstimatedRobotPoses of the bot relative to the field
     *         (even null ones!)
     */
    public static List<EstimatedRobotPose> getGlobalFieldPoses() {

        List<EstimatedRobotPose> poses = new ArrayList<>();

        for (int i = 0; i < numberOfCams; i++) {
            EstimatedRobotPose pose = getGlobalFieldPoseForDrivetrain(cameras[i], poseEstimators[i]);

            poses.add(pose);
        }

        return poses;
    }

    /**
     * 
     * @param camera,              a PhotonVision camera
     * @param photonPoseEstimator, a PhotonPoseEstimator with the matching
     *                             translation for the camera
     * @return the EstimatedRobotPose of the bot relative to the field
     */
    private static EstimatedRobotPose getGlobalFieldPoseForDrivetrain(PhotonCamera camera,
            PhotonPoseEstimator photonPoseEstimator) {
        if (!canSeeTag(camera)) {
            return null;
        }

        PhotonPipelineResult result = camera.getLatestResult();

        if (result != null && result.hasTargets()) {
            Optional<EstimatedRobotPose> estimatedRobotPose = photonPoseEstimator.update(result);

            if (estimatedRobotPose.isPresent()) {
                return estimatedRobotPose.get();
            }
        }

        return null;
    }
}