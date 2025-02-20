package frc.robot.commands.auto;

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

    private double xShift;
    private double yShift;

    private boolean doneMoving;
    private boolean doneRotating;

    public ExactAlign(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;

        this.xShift = Constants.MIN_CAMERA_DISTANCE;
        this.yShift = 0.0;
    }

    public ExactAlign(CommandSwerveDrivetrain drivetrain, double xShift, double yShift) {
        this.drivetrain = drivetrain;

        this.xShift = xShift;
        this.yShift = yShift;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        doneMoving = false;
        doneRotating = false;
    }

    @Override
    public void execute() {
        double[] raw = NetworkTableInstance.getDefault().getTable("limelight-left").getEntry("targetpose_robotspace").getDoubleArray(new double[6]);
        targetPose = new Pose3d(raw[0] + xShift, raw[1] + yShift, raw[2], new Rotation3d(raw[4], raw[5], raw[3])).toPose2d();


        double distanceToTarget = (new Translation2d(0, 0)).getDistance(targetPose.getTranslation());
        if (distanceToTarget < Constants.GENERIC_DISTANCE_THRESHOLD) {doneMoving = true;}

        double rotationToTarget = Math.abs(targetPose.getRotation().getRadians());
        if (rotationToTarget < Constants.GENERIC_ROTATION_THRESHOLD) {doneRotating = true;}


        double velocityX = 0;
        double velocityY = 0;
        double rotationalRate = 0;

        if (!doneMoving) {
            velocityX = targetPose.getX() * Constants.ALIGN_MOVE_SPEED;
            velocityY = targetPose.getY() * Constants.ALIGN_MOVE_SPEED;
        }
        if (!doneRotating) {
          double rotationError = targetPose.getRotation().getRadians();
          double RESign = rotationError / Math.abs(rotationError);
          rotationalRate = rotationError * Constants.ALIGN_ROTATE_SPEED
          + (RESign * Constants.ALIGN_ROTATIONAL_FEED_FORWARD);
        }

        System.out.println(targetPose.getX() + " " + targetPose.getY() + " " + targetPose.getRotation().getRadians());

        if (!(doneMoving && doneRotating)) {
          drivetrain.setControl(driveRequest.withVelocityX(-velocityX).withVelocityY(-velocityY).withRotationalRate(-rotationalRate));
        }
    }

    @Override
    public boolean isFinished() {
        return doneMoving && doneRotating;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Alignment complete!");
        drivetrain.setControl(new SwerveRequest.SwerveDriveBrake());
    }

}