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
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;

enum CameraType {
    APRIL_TAG, CORAL, ALGAE
}

public class VisionSubsystem {

    private static int numberOfCams = Constants.numberOfCams;

    private static PhotonCamera[] cameras = new PhotonCamera[numberOfCams];
    private static CameraType[] cameraTypes = new CameraType[numberOfCams];

    private static PhotonPoseEstimator[] poseEstimators = new PhotonPoseEstimator[numberOfCams];
    private static Transform3d[] cameraToBotRelativePoses = { //REALLY USEFUL DOCS FOR COORDINATE SYSTEMS: https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html
            new Transform3d(0.281, -0.176,  0.265, new Rotation3d(0, 0, 0)),
            new Transform3d(0.281, 0.176,  0.265, new Rotation3d(0, 0, 0))
    };

    private static AprilTagFieldLayout fieldLayout;

    /**
     * Initializes the vision subsystem with the proper PhotonCameras and
     * photonPoseEstimators
     */
    public static void initializeVisionSubsystem() {
        System.out.println(NetworkTableInstance.getDefault());
        loadField();

        for (int i = 0; i < numberOfCams; i++) {
            PhotonCamera c = new PhotonCamera(NetworkTableInstance.getDefault(), "camera" + i);

            PhotonPoseEstimator e = new PhotonPoseEstimator(
                fieldLayout, 
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                cameraToBotRelativePoses[i]
            );
            e.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

            CameraType t = pipeLineAsCameraType(c.getPipelineIndex());
            System.out.println("pipeline num = " + c.getPipelineIndex());

            cameras[i] = c;
            poseEstimators[i] = e;
            cameraTypes[i] = t;
        }
    }

    /**
     * Loads the field
     */
    private static void loadField() {
        fieldLayout = AprilTagFieldLayout.loadField(Constants.FIELD_LAYOUT);
    }

    /**
     * 
     * @param pipeLineValue, the passed pipeline value of the camera
     * @return the corresponding camera type (this will change every year)
     */
    private static CameraType pipeLineAsCameraType(int pipeLineValue) {
        if (pipeLineValue == 0) {
            return CameraType.APRIL_TAG;
        }
        if (pipeLineValue == 1) {
            return CameraType.CORAL;
        }
        if (pipeLineValue == 2) {
            return CameraType.ALGAE;
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
     * @param poses, an arraylist of pose3ds
     * @return the average of all of these poses (including the transpose), or null if all poses are null
     */
    public static Pose3d averagePoses(ArrayList<Pose3d> poses) {
        double x = 0;
        double y = 0;
        double z = 0;
        double roll = 0;
        double pitch = 0;
        double yaw = 0;
        int count = 0;

        for (int i = 0; i < poses.size(); i++) {
            Pose3d pose = poses.get(i);

            if (pose != null) {
                x += pose.getX() + cameraToBotRelativePoses[i].getX();
                y += pose.getY() + cameraToBotRelativePoses[i].getY();
                z += pose.getZ() + cameraToBotRelativePoses[i].getZ();

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
     * @return A list of all EstimatedRobotPoses of the bot relative to the field
     *         (even null ones!)
     */
    public static List<EstimatedRobotPose> getGlobalFieldPoses() {

        List<EstimatedRobotPose> poses = new ArrayList<>();

        ;

        for (int i = 0; i < numberOfCams; i++) {
            EstimatedRobotPose pose = getGlobalFieldPoseForDrivetrain(cameras[i], poseEstimators[i]);
/* 
            if (pose != null) {
                System.out.println("Camera " + i + " sees tags! " + pose.estimatedPose);
            } else {
                System.out.println("Camera " + i + " sees no tags.");
            } */
    
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

    /**
     * 
     * @param priorityTagID, the priority tag ID at which to specifically look for, pass -1 for Photon choice
     * @return the average Pose3d of the specified apriltag relative to the bot for
     *         all of the cameras
     */
    public static ArrayList<Pose3d> getTagRelativeToBot(int priorityTagID) {
        ArrayList<Pose3d> poses = new ArrayList<>();

        for (int i = 0; i < numberOfCams; i++) {
            if (cameraTypes[i] != CameraType.APRIL_TAG) {
                poses.add(null);
                continue;
            }

            poses.add(getTagRelativeToBot(cameras[i], priorityTagID));
        }

        return poses;
    }

    /**
     * 
     * @param camera,        a PhotonVision camera
     * @param priorityTagID, the priority tag ID at which to specifically look for, pass -1 for Photon choice
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

            if (priorityTagID == -1) {
                Transform3d transform3d = result.getBestTarget().getBestCameraToTarget();

                Translation3d translation3d = transform3d.getTranslation();
                Rotation3d rotation3d = transform3d.getRotation();

                return new Pose3d(translation3d, rotation3d);
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
     * @param priorityTagID, the priority tag ID at which to specifically look for, pass -1 for Photon choice
     * @return the average Pose3d of the bot relative to the specified apriltag for
     *         all of the cameras
     */
    public static ArrayList<Pose3d> getBotRelativeToTag(int priorityTagID) {
        ArrayList<Pose3d> poses = new ArrayList<>();

        for (int i = 0; i < numberOfCams; i++) {
            if (cameraTypes[i] != CameraType.APRIL_TAG) {
                poses.add(null);
                continue;
            }
            
            poses.add(getBotRelativeToTag(cameras[i], priorityTagID));
        }

        return poses;
    }

    /**
     * 
     * @param camera,        a PhotonVision camera
     * @param priorityTagID, the priority tag ID at which to specifically look for, pass -1 for Photon choice
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

            if (priorityTagID == -1) {
                Transform3d transform3d = result.getBestTarget().bestCameraToTarget;

                Translation3d translation3d = transform3d.getTranslation();
                Rotation3d rotation3d = transform3d.getRotation();

                return new Pose3d(new Translation3d(-translation3d.getX(), -translation3d.getY(), -translation3d.getZ()), rotation3d);
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
     * @param priorityTagID, the priority tag ID at which to specifically look for, pass -1 for Photon choice
     * @return the average distance to the specified apriltag for all of the cameras
     */
    public static double getDistanceToAprilTag(int priorityTagID) {
        double totalDistance = 0;
        int count = 0;

        for (int i = 0; i < numberOfCams; i++) {
            if (cameraTypes[i] != CameraType.APRIL_TAG) {
                continue;
            }

            double distance = getDistanceToAprilTag(cameras[i], priorityTagID);

            if (distance != -1) {
                totalDistance += distance + cameraToBotRelativePoses[i].getX();
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
     * @param priorityTagID, the priority tag ID at which to specifically look for, pass -1 for Photon choice
     * @return the distance to the specified apriltag
     */
    private static double getDistanceToAprilTag(PhotonCamera camera, int priorityTagID) {
        PhotonPipelineResult result = camera.getLatestResult();

        if (result != null && result.hasTargets()) {
            List<PhotonTrackedTarget> targets = result.targets;

            if (targets.size() == 0) {
                return -1;
            }

            if (priorityTagID == -1) {
                return result.getBestTarget().getBestCameraToTarget().getTranslation().getNorm();
            }

            for (PhotonTrackedTarget target : targets) {
                if (target.getFiducialId() == priorityTagID) {
                    return target.getBestCameraToTarget().getTranslation().getNorm();
                }
            }
        }

        return -1;
    }
}