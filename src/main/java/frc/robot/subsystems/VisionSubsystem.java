package frc.robot.subsystems;

import java.io.File;
import java.lang.StackWalker.Option;
import java.nio.file.Path;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagPoseEstimator;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;

//for alignment
import edu.wpi.first.math.controller.PIDController;

public class VisionSubsystem {

    private int numberOfCams;

    private PhotonCamera[] cameras;
    private Field2d field;
    public AprilTagFieldLayout aprilTagFieldLayout;
    private PhotonPoseEstimator photonPoseEstimator;
    private int priorityTagID;

    // private Transform3d robotToCam;
    // private Matrix<N3, N1> curStdDevs;
    // private static Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4,0,8);
    // private static Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(4,0,8);

    // private VisionSystemSim visionSim;

    public VisionSubsystem(int numberOfCams) { // perhaps remove the drivetrain if not using
        this.numberOfCams = numberOfCams;

        System.out.println(NetworkTableInstance.getDefault());

        cameras = new PhotonCamera[numberOfCams];

        for (int i = 0; i < numberOfCams; i++) {
            cameras[i] = new PhotonCamera(NetworkTableInstance.getDefault(), "camera" + i);
        }
        field = new Field2d();

        try {
            Path path = Filesystem.getDeployDirectory().toPath().resolve("AprilTag2025Layout.json");
            aprilTagFieldLayout = new AprilTagFieldLayout(path);
        } catch (Exception e) {
            System.out.println("April tags failed to initialize!");
        }
    }

    public void setLabel(Pose2d pose2d, String label) {
        field.getObject(label).setPose(pose2d);
    }

    public double getDistanceToAprilTag(boolean usePriority, boolean strict) {
        double totalDistance = 0;
        int count = 0;

        for (int i = 0; i < numberOfCams; i++) {
            double distance = getDistanceToAprilTag(cameras[i], usePriority, strict);
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

    private double getDistanceToAprilTag(PhotonCamera camera, boolean usePriority, boolean strict) {
        PhotonPipelineResult result = camera.getLatestResult();

        if (result != null && result.hasTargets()) {

            List<PhotonTrackedTarget> targets = result.targets;

            if (targets.size() == 0) {
                return -1;
            }

            if (usePriority) {
                for (PhotonTrackedTarget target : targets) {
                    if (target.getFiducialId() == priorityTagID) {
                        return target.getBestCameraToTarget().getTranslation().getNorm();
                    }
                }
                if (strict) {
                    return -1;
                }
            }

            PhotonTrackedTarget target = result.getBestTarget();
            Transform3d transform = target.getBestCameraToTarget();

            return transform.getTranslation().getNorm();
        }

        return -1;
    }

    public Pose3d getTagRelativeToBot(boolean usePriority, boolean strict) {
        double x = 0;
        double y = 0;
        double z = 0;
        double roll = 0;
        double pitch = 0;
        double yaw = 0;
        int count = 0;

        for (int i = 0; i < numberOfCams; i++) {
            Pose3d pose = getTagRelativeToBot(cameras[i], usePriority, strict);
            
            if (pose != null) {
                x += pose.getX();
                y += pose.getY();
                z += pose.getZ();

                roll += pose.getRotation().getX();
                pitch += pose.getRotation().getY();
                yaw += pose.getRotation().getZ();
            }
        }

        if (count == 0) {
            return null;
        }

        return new Pose3d(x / count, y / count, z / count, new Rotation3d(roll / count, pitch / count, yaw / count));
    }

    private Pose3d getTagRelativeToBot(PhotonCamera camera, boolean usePriority, boolean strict) {
        if (!canSeeTag(camera)) {
            return null;
        }

        PhotonPipelineResult result = camera.getLatestResult();

        if (result != null && result.hasTargets()) {

            List<PhotonTrackedTarget> targets = result.targets;

            if (targets.size() == 0) {
                return null;
            }

            if (usePriority) {
                for (PhotonTrackedTarget target : targets) {
                    if (target.getFiducialId() == priorityTagID) {
                        Transform3d transform3d = target.getBestCameraToTarget();
                        Translation3d translation3d = transform3d.getTranslation();
                        Rotation3d rotation3d = transform3d.getRotation();

                        return new Pose3d(translation3d, rotation3d);
                    }

                }
                if (strict) {
                    return null;
                }
            }

            Transform3d transform3d = result.getBestTarget().bestCameraToTarget;

            Translation3d translation3d = transform3d.getTranslation();
            Rotation3d rotation3d = transform3d.getRotation();

            return new Pose3d(translation3d, rotation3d);
        } else {
            return null;
        }
    }

    public Pose3d getBotRelativeToTag(boolean usePriority, boolean strict) {
        double x = 0;
        double y = 0;
        double z = 0;
        double roll = 0;
        double pitch = 0;
        double yaw = 0;
        int count = 0;

        for (int i = 0; i < numberOfCams; i++) {
            Pose3d pose = getBotRelativeToTag(cameras[i], usePriority, strict);
            
            if (pose != null) {
                x += pose.getX();
                y += pose.getY();
                z += pose.getZ();

                roll += pose.getRotation().getX();
                pitch += pose.getRotation().getY();
                yaw += pose.getRotation().getZ();
            }
        }

        if (count == 0) {
            return null;
        }

        return new Pose3d(x / count, y / count, z / count, new Rotation3d(roll / count, pitch / count, yaw / count));
    }

    private Pose3d getBotRelativeToTag(PhotonCamera camera, boolean usePriority, boolean strict) {
        if (!canSeeTag(camera)) {
            return null;
        }

        PhotonPipelineResult result = camera.getLatestResult();

        if (result != null && result.hasTargets()) {

            List<PhotonTrackedTarget> targets = result.targets;

            if (targets.size() == 0) {
                return null;
            }

            if (usePriority) {
                for (PhotonTrackedTarget target : targets) {
                    if (target.getFiducialId() == priorityTagID) {
                        Transform3d transform3d = target.getBestCameraToTarget();
                        Translation3d translation3d = transform3d.getTranslation();
                        Rotation3d rotation3d = transform3d.getRotation();

                        return new Pose3d(translation3d, rotation3d);
                    }

                }
                if (strict) {
                    return null;
                }
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

    private boolean canSeeTag(PhotonCamera camera) {
        return camera.getLatestResult().hasTargets();
    }

    // public int getTagID() {
    //     if (canSeeTag()) {
    //         PhotonPipelineResult latestResult = camera.getLatestResult();
    //         if (latestResult != null && latestResult.hasTargets()) {
    //             PhotonTrackedTarget target = latestResult.getBestTarget();
    //             return target.getFiducialId();
    //         }
    //     }
    //     return -1;
    // }

    public void setPriorityTagID(int id) {
        priorityTagID = id;
    }

    // public PhotonPoseEstimator getPhotonPoseEstimator() {
    // return new PhotonPoseEstimator(aprilTagFieldLayout,
    // PoseStrategy.LOWEST_AMBIGUITY, transform3d);
    // }

    // public PhotonPipelineResult getLatestPhotonPipelineResult() {
    // return camera.getLatestResult();
    // }
}