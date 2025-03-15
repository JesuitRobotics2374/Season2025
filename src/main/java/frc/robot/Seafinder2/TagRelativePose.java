package frc.robot.Seafinder2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class TagRelativePose {
    int tagId;

    // NWU coordinate system
    double x;
    double y;
    double yaw;

    public TagRelativePose(int tagId, double x, double y, double yaw) {
        this.tagId = tagId;
        this.x = x;
        this.y = y;
        this.yaw = yaw;
    }

    public int getTagId() {
        return tagId;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getYaw() {
        return yaw;
    }

    public Pose2d getPose2d() {
        return new Pose2d(x, y, new Rotation2d(yaw));
    }

    public String toString() {
        return "Tag " + tagId + " at (" + x + ", " + y + ") with yaw " + yaw;
    }
}