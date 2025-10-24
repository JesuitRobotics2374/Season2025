package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class VisionSubsystem extends SubsystemBase{

    private Camera[] cameras; // An array holding the PhotonCamera instances
    private AprilTagFieldLayout aprilTagFieldLayout; // The AprilTag field layout

    private int numCams = Constants.numberOfCams;
    
    public VisionSubsystem() {
        loadField();

        for (int i = 0; i < numCams; i++) {
            cameras[i] = new Camera(); //TODO: CONSTRUCTOR
        }



        // TODO: CONSTRUCTOR
    }

    private void loadField() {
        try {
            aprilTagFieldLayout = new AprilTagFieldLayout(AprilTagFieldLayout.); //TODO: GET FIELD
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public AprilTagFieldLayout getField() {
        return aprilTagFieldLayout;
    }

    public List<Integer> getAllVisibleTags() {
        List<Integer> allTags = new ArrayList<>();

        for (int i = 0; i < numCams; i++) {
            allTags.addAll(cameras[i].getAllAvailableTags());
        }

        return allTags;
    }

    public boolean canSeeTag(int tagID) {
        List<Integer> allTags = getAllVisibleTags();
        
        if (tagID == -1) {
            return allTags.size() > 0;
        }

        return allTags.contains(tagID);
    }

    public Pose3d averagePoses(ArrayList<Pose3d> poses) {
        double x = 0;
        double y = 0;
        double z = 0;
        double rotX = 0;
        double rotY = 0;
        double rotZ = 0;
        double count = 0;

        for (Pose3d pose : poses) {
            if (pose == null) {
                continue;
            }

            x += pose.getX();
            y += pose.getY();
            z += pose.getZ();
            rotX += pose.getRotation().getX();
            rotY += pose.getRotation().getY();
            rotZ += pose.getRotation().getZ();

            count++;
        }

        if (count == 0) {
            return null;
        }

        return new Pose3d(new Translation3d(x / count, y / count, z / count), 
                          new Rotation3d(rotX / count, rotY / count, rotZ / count));
    }

    public List<EstimatedRobotPose> getGlobalFieldPoses() {
        List<EstimatedRobotPose> allPoses = new ArrayList<>();

        for (int i = 0; i < numCams; i++) {
            if (cameras[i].getCameraType() != Camera.Type.APRIL_TAG) {
                continue;
            }

            allPoses.add(cameras[i].getGlobalFieldPose());
        }

        return allPoses;
    }

    public Pose3d getTagRelativeToBot(int tagID) {
        ArrayList<Pose3d> tagPoses = new ArrayList<>();

        for (int i = 0; i < numCams; i++) {

            if (cameras[i].getCameraType() != Camera.Type.APRIL_TAG) {
                continue;
            }

            tagPoses.add(cameras[i].getTagRelativeToBot(tagID));
        }

        return averagePoses(tagPoses);
    }

    public Pose3d getBotRelativeToTag(int tagID) {
        ArrayList<Pose3d> tagPoses = new ArrayList<>();

        for (int i = 0; i < numCams; i++) {

            if (cameras[i].getCameraType() != Camera.Type.APRIL_TAG) {
                continue;
            }

            tagPoses.add(cameras[i].getBotRelativeToTag(tagID));
        }

        return averagePoses(tagPoses);
    }

    public Pose3d getNearestTag() {
        List<Integer> visibleTags = getAllVisibleTags();

        List<Pose3d> tagPoses = new ArrayList<>();

        for (int tagID : visibleTags) {
            Pose3d tagPose = getTagRelativeToBot(tagID);

            if (tagPose != null) {
                tagPoses.add(tagPose);
            }
        }

        if (tagPoses.size() == 0) {
            return null;
        }

        Pose3d nearestTag = tagPoses.get(0);

        for (Pose3d pose : tagPoses) {
            if (pose.getTranslation().getNorm() < nearestTag.getTranslation().getNorm()) {
                nearestTag = pose;
            }
        }

        return nearestTag;
    }

    public double getDistanceToTag(int tagID) {
        Pose3d tagPose = getTagRelativeToBot(tagID);

        if (tagPose == null) {
            return -1;
        }

        return tagPose.getTranslation().getNorm();
    }
}
