package frc.robot.seafinder.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;

public class ExactAlign extends Command {

    private final SwerveRequest.RobotCentric driveRequest = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final CommandSwerveDrivetrain drivetrain;
    private Pose2d targetPose;

    private double alignmentShift;

    private boolean doneMoving;
    private boolean doneRotating;

    private int tag_id;

    private int clock = 0;

    public ExactAlign(CommandSwerveDrivetrain drivetrain, int tag_id) {
        this.drivetrain = drivetrain;
        
        this.alignmentShift = 0.0;

        this.tag_id = tag_id;
    }

    public ExactAlign(CommandSwerveDrivetrain drivetrain, int tag_id, double alignmentShift) {
        this.drivetrain = drivetrain;

        this.alignmentShift = alignmentShift;

        this.tag_id = tag_id;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        doneMoving = false;
        doneRotating = false;
    }

    @Override
    public void execute() {
        doneRotating = true; // TODO: REMOVE

        if (NetworkTableInstance.getDefault().getTable("limelight-left").getEntry("tid").getDouble(-1) != tag_id) {
            System.out.println("No target found: " + tag_id + " vs " + NetworkTableInstance.getDefault().getTable("limelight-left").getEntry("tid").getDouble(-1));
            System.out.println("No taret found");
            System.out.println("No taret found");
            doneMoving = true;
            doneRotating = true;
            return;
        }


        double[] raw = NetworkTableInstance.getDefault().getTable("limelight-left").getEntry("targetpose_robotspace").getDoubleArray(new double[6]);
        raw[0] += alignmentShift;
        
        targetPose = new Pose3d(raw[0], raw[1], raw[2], new Rotation3d(raw[5], raw[3], raw[4])).toPose2d();
        // Logging
        System.out.println("XY" + raw[0] + " " + raw[1]);
        Pose2d robotPose = drivetrain.getEstimator();
        Pose2d targetFieldRelPose2d = robotPose.relativeTo(targetPose);
        drivetrain.setLabel(targetFieldRelPose2d, "Target Field Rel Pose");
        // Logging

        double distanceToTarget = (new Translation2d(0, 0)).getDistance(targetPose.getTranslation());
        if (distanceToTarget < Constants.GENERIC_DISTANCE_THRESHOLD) {doneMoving = true;}

        double rotationToTarget = Math.abs(targetPose.getRotation().getRadians());
        if (rotationToTarget < Constants.GENERIC_ROTATION_THRESHOLD) {doneRotating = true;}


        double velocityX = 0;
        double velocityY = 0;
        double magnitude = Math.sqrt(Math.pow(raw[0], 2) + Math.pow(raw[1], 2)); // TODO: Retune after removing magnitude
        double rotationalRate = 0;

        if (!doneMoving) {
            velocityX = raw[0] / (magnitude + 1e-6) * Constants.ALIGN_MOVE_SPEED;
            velocityY = raw[1] / (magnitude + 1e-6) * Constants.ALIGN_MOVE_SPEED;
        }
        if (!doneRotating) {
          double rotationError = targetPose.getRotation().getRadians();
          double RESign = rotationError / Math.abs(rotationError);
          rotationalRate = rotationError * Constants.ALIGN_ROTATE_SPEED
          + (RESign * Constants.ALIGN_ROTATIONAL_FEED_FORWARD);
        }

        System.out.println((new Translation2d(0, 0)).getDistance(targetPose.getTranslation()));
        System.out.println("DIS" + distanceToTarget + " VELX: " + velocityX + " VELY: " + velocityY + " ROT: " + rotationalRate);

        // + Y is Forard on dr 
        if (!(doneMoving && doneRotating)) {
          drivetrain.setControl(driveRequest.withVelocityX(velocityY).withVelocityY(-velocityX)); //.withRotationalRate(-rotationalRate));
        }
    }

    @Override
    public boolean isFinished() {
        System.out.println("--" + doneMoving + " " + doneRotating + "--");
        return doneMoving && doneRotating;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("ExactAlign ended" + interrupted);
        drivetrain.setControl(new SwerveRequest.SwerveDriveBrake());
    }

}