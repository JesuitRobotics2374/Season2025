package frc.robot.seafinder2.utils;

import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;

public class Apriltags {

    private static AprilTagFieldLayout fieldLayout;

    public static void loadField() {
        fieldLayout = AprilTagFieldLayout
                .loadField(AprilTagFields.k2025ReefscapeWelded);
    }

    public static Pose3d getWeldedPosition(int tagId) { // Note: The welded map is used because all events 2374 will
                                                        // take part in happen on welded fields as opposed to Andymark
                                                        // fields.
        final Optional<Pose3d> pose = fieldLayout.getTagPose(tagId);
        if (pose.isPresent()) {
            return pose.get();
        }
        return null;
    }

}
