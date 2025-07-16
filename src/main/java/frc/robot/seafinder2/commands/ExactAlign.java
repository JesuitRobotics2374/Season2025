package frc.robot.seafinder2.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import java.util.ArrayList;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.seafinder2.utils.Target.TagRelativePose;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;

public class ExactAlign extends Command {

    private final SwerveRequest.RobotCentric driveRequest = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    // PID Controllers for x, y, and rotation
    private final PIDController xController;
    private final PIDController yController;
    private final PIDController yawController;

    // Rate limiters for smoother motion
    private final SlewRateLimiter xRateLimiter = new SlewRateLimiter(2.0);
    private final SlewRateLimiter yRateLimiter = new SlewRateLimiter(2.0);
    private final SlewRateLimiter yawRateLimiter = new SlewRateLimiter(100.0);

    // Position tolerance thresholds
    private static final double X_TOLERANCE = 0.035; // meters
    private static final double Y_TOLERANCE = 0.035; // meters
    private static final double YAW_TOLERANCE = 3 * Math.PI / 180; // radians

    // Maximum output valuess
    private static final double MAX_LINEAR_SPEED = 3.4;
    private static final double MAX_ANGULAR_SPEED = 0.5;

    private static final double X_SPEED_MODIFIER = 1;
    private static final double Y_SPEED_MODIFIER = 0.75;
    private static final double THETA_SPEED_MODIFIER = 0.75;

    // Minimum output to overcome static friction
    private static final double MIN_LINEAR_COMMAND = 0.17;
    private static final double MIN_ANGULAR_COMMAND = 0.25;

    // State tracking
    private int framesAtTarget = 0;
    private static final int REQUIRED_FRAMES_AT_TARGET = 25;
    private int framesWithoutTarget = 0;
    private static final int MAX_FRAMES_WITHOUT_TARGET = 10;
    private static final int MAX_FRAMES_BEFORE_REDUCE = 3;
    private static final int MAX_FRAMES_TO_AVERAGE = 7;

    private final CommandSwerveDrivetrain drivetrain;
    // private final SwerveRequest.RobotCentric drive;

    private final int tagId;
    private final double x_offset;
    private final double y_offset;
    private final double yaw_offset;

    private double dx;
    private double dy;
    private double dtheta;

    private ArrayList<Pose3d> recentPoses;

    boolean finishedOverride;

    public ExactAlign(CommandSwerveDrivetrain drivetrain, TagRelativePose tagRelativePose) {

        finishedOverride = false;
//        private int clock = 0;

        this.drivetrain = drivetrain;
        this.tagId = tagRelativePose.getTagId();
        this.x_offset = tagRelativePose.getX();
        this.y_offset = tagRelativePose.getY();
        this.yaw_offset = tagRelativePose.getYaw();

        recentPoses = new ArrayList<>();

        // Initialize PID controllers
        // X PID coefficients (Adjust these values based on testing)
        xController = new PIDController(2.5, 0.0, 1.8);
        xController.setTolerance(X_TOLERANCE);

        // Y PID coefficients
        yController = new PIDController(3, 0.0, 2.3);
        yController.setTolerance(Y_TOLERANCE);

        // Yaw PID coefficients
        yawController = new PIDController(2.2, 0.1, 0.2);
        yawController.setTolerance(YAW_TOLERANCE);
        yawController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void initialize() {

        finishedOverride = false;

        System.out.println("EXACTALIGN STARTED");

        // Reset controllers and rate limiters
        xController.reset();
        yController.reset();
        yawController.reset();
        xRateLimiter.reset(0);
        yRateLimiter.reset(0);
        yawRateLimiter.reset(0);

        framesAtTarget = 0;
        framesWithoutTarget = 0;
    }



    @Override
    public void execute() {
        drivetrain.setControl(driveRequest
                    .withVelocityX(-dx)
                    .withVelocityY(-dy)
                    .withRotationalRate(dtheta)
                );

      //  clock++;

        // Average pose from each camera
        double avg_x = 0;
        double avg_y = 0;
        double avg_yaw = 0;

        dx = 0;
        dy = 0;
        dtheta = 0;

        Pose3d currentPose = VisionSubsystem.averagePoses(VisionSubsystem.getTagRelativeToBot(tagId));
        Pose3d usePose = null;

        if (currentPose == null) {
            System.out.println("Pose Null - Frames w/out vision: "+ framesWithoutTarget);

            framesWithoutTarget++;

            if (framesWithoutTarget > MAX_FRAMES_WITHOUT_TARGET) {
                end(true);
            }

            // Maintain last movement but slowly reduce it
            if (framesWithoutTarget > MAX_FRAMES_BEFORE_REDUCE) {
                double xRate = driveRequest.VelocityX;
                double yRate = driveRequest.VelocityY;
                double rRate = driveRequest.RotationalRate;

                if (Math.abs(recentPoses.get(recentPoses.size() - 1).getX() + x_offset) < 1) {
                    xRate = xRateLimiter.calculate(0);
                    rRate = yawRateLimiter.calculate(0);
                    System.out.println("reducing x");
                }
                if (Math.abs(recentPoses.get(recentPoses.size() - 1).getY() + y_offset) < 0.1) {
                    yRate = yRateLimiter.calculate(0);
                    rRate = yawRateLimiter.calculate(0);
                    System.out.println("reducing y");
                }

                dx = xRate;
                dy = yRate;
                dtheta = rRate;
            }
            return;
        }

        framesWithoutTarget = 0;

        if (recentPoses.size() >= MAX_FRAMES_TO_AVERAGE) {
            recentPoses.remove(0);
        }

        recentPoses.add(currentPose);
        usePose = averagePoses(recentPoses);

        framesWithoutTarget = 0;
        avg_x = usePose.getX();
        avg_y = usePose.getY();
        avg_yaw = usePose.getRotation().getZ();

        if (avg_yaw < 0) {
            avg_yaw = -Math.abs(Math.PI + avg_yaw);
        } else {
            avg_yaw = Math.abs(avg_yaw - Math.PI);
        }

        // Calculate errors (target offset - current position)
        double error_x = avg_x - x_offset;
        double error_y = avg_y - y_offset;
        double error_yaw = avg_yaw - yaw_offset;

        // Normalize yaw error to -π to π range
        error_yaw = Rotation2d.fromRadians(error_yaw).getRadians();

        // Calculate PID outputs
        dx = xController.calculate(avg_x, x_offset);
        dy = yController.calculate(avg_y, y_offset);
        dtheta = yawController.calculate(avg_yaw, yaw_offset);

        // Apply minimum command if needed
        if (Math.abs(error_x) > X_TOLERANCE && Math.abs(dx) < MIN_LINEAR_COMMAND) {
            dx = MIN_LINEAR_COMMAND * Math.signum(dx);
        }

        if (Math.abs(error_y) > Y_TOLERANCE && Math.abs(dy) < MIN_LINEAR_COMMAND) {
            dy = MIN_LINEAR_COMMAND * Math.signum(dy);
        }

        if (Math.abs(error_yaw) > YAW_TOLERANCE && Math.abs(dtheta) < MIN_ANGULAR_COMMAND) {
            dtheta = MIN_ANGULAR_COMMAND * Math.signum(dtheta);
        }

        // Limit outputs to maximum values
        dx = Math.max(-MAX_LINEAR_SPEED, Math.min(dx * X_SPEED_MODIFIER, MAX_LINEAR_SPEED));
        dy = Math.max(-MAX_LINEAR_SPEED, Math.min(dy * Y_SPEED_MODIFIER, MAX_LINEAR_SPEED));
        dtheta = Math.max(-MAX_ANGULAR_SPEED, Math.min(dtheta * THETA_SPEED_MODIFIER, MAX_ANGULAR_SPEED));

        // Apply rate limiting for smoother motion
        dx = xRateLimiter.calculate(dx);
        dy = yRateLimiter.calculate(dy);
        dtheta = yawRateLimiter.calculate(dtheta);

        // Zero out commands if we're within tolerance
        boolean xTollerenace = Math.abs(error_x) < X_TOLERANCE;
        boolean yTollerenace = Math.abs(error_y) < Y_TOLERANCE;
        boolean thetaTollerenace = Math.abs(error_yaw) + (0.5 * Math.PI / 180) < YAW_TOLERANCE;

        if (xTollerenace)
            dx = 0;
        if (yTollerenace)
            dy = 0;
        if (thetaTollerenace)
            dtheta = 0;

        // Set the drive request
      //  if (clock >= 20) {
      //      System.out.println("Drive Control:  dx: " + dx + " dy: " + dy);
      //  }


        // Update state for isFinished
        if (xTollerenace && yTollerenace && thetaTollerenace) {
            framesAtTarget++;
        } else {
            framesAtTarget = 0;
        }


    }

    @Override
    public void end(boolean interrupted) {
        finishedOverride = true;
        drivetrain.setControl(new SwerveRequest.SwerveDriveBrake());
        // drivetrain.setDefaultCommand(
        //         drivetrain.applyRequest(() -> drive
        //                 .withVelocityX(0)
        //                 .withVelocityY(0)
        //                 .withRotationalRate(0)));
        if (interrupted) {
            System.out.println("EXACTALIGN INTERRUPTED");
        } else {
            System.out.println("EXACTALIGN FINISHED");
        }
    }

    @Override
    public boolean isFinished() {
        return framesAtTarget >= REQUIRED_FRAMES_AT_TARGET || finishedOverride;
    }

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
}