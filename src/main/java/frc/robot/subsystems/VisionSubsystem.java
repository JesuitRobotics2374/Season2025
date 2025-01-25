package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.LimelightHelpers;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;

public class VisionSubsystem extends SubsystemBase {

    private static VisionSubsystem instance;
    // ShuffleboardTab tab = Shuffleboard.getTab(Constants.DRIVER_READOUT_TAB_NAME);

    private int offset = 5000;

    /** Creates a new VisionSubsystem. */
    public VisionSubsystem() {

        instance = this;

        LimelightHelpers.setLEDMode_PipelineControl("limelight-left");
        LimelightHelpers.setLEDMode_ForceBlink("limelight-left");

    }

    public class DistanceAndAngle {
        private final double distance;
        private final double theta;

        public DistanceAndAngle(double distance, double theta) {

            this.distance = distance;
            this.theta = theta;
        }

        public double getDistance() {
            return distance;
        }

        public double getTheta() {
            return theta;
        }

        public double getDistanceMeters() {
            return distance / 39.37;
        }

        @Override
        public String toString() {
            return String.format("Distance: %.2f inches, Angle: %.2f degrees", distance, theta);
        }
    }

    public boolean canSeeTag(int tag_id) {
        int detectedTagId = (int) LimelightHelpers.getFiducialID("limelight-left");
        return (detectedTagId == tag_id);
    }

    public DistanceAndAngle getTagDistanceAndAngle(int tag_id) {

        int detectedTagId = (int) LimelightHelpers.getFiducialID("limelight-left");
        if (detectedTagId == tag_id) {

            double tagHeight = LimelightHelpers.getT2DArray("limelight-left")[15];
            double tagWidth = LimelightHelpers.getT2DArray("limelight-left")[14];

            double tx = LimelightHelpers.getTX("limelight-left"); // Horizontal angle offset
            System.out.println("TX: " + tx);
            double xRot = Math.acos(tagWidth / tagHeight);
            // System.out.println("calc xrot: " + xRot * (180 / Math.PI));

            // double ta = LimelightHelpers.getTA("limelight-left"); // Tag screen coverage

            double distance = offset / tagHeight; // inches

            return new DistanceAndAngle(distance, xRot);
        }

        return new DistanceAndAngle(-1.0, -1.0);
    }

    public Pose3d getTagPose3d(int tag_id) {
        int detectedTagId = (int) LimelightHelpers.getFiducialID("limelight-left");
        if (detectedTagId == tag_id) {
            return LimelightHelpers.getTargetPose3d_CameraSpace("limelight-left");
        }
        return null;
    }

    public void raiseOffset() {
        offset += 5;
        System.out.println(offset);
    }

    public void lowerOffset() {
        offset -= 5;
        System.out.println(offset);
    }

    public void grabMisc(int tag_id) {
        int detectedTagId = (int) LimelightHelpers.getFiducialID("limelight-left");
        if (detectedTagId == tag_id) {

            double tagHeight = LimelightHelpers.getT2DArray("limelight-left")[15];
            double tagWidth = LimelightHelpers.getT2DArray("limelight-left")[14];
            DistanceAndAngle d = getTagDistanceAndAngle(tag_id);

            double f = (Math.PI / 2) - d.getTheta() - Math.acos(tagWidth / tagHeight);

            System.out.println("resultant: " + f);
        }
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public void driveStatic(CommandSwerveDrivetrain m_DrivetrainSubsystem, int testTargetTag) {
        // DriveDynamic drive = new DriveDynamic(m_DrivetrainSubsystem, 0.2);
        // driveDynamic = drive;
        // drive.schedule();
    }
}