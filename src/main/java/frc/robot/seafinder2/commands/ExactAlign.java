package frc.robot.seafinder2.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
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
    private final SlewRateLimiter xRateLimiter = new SlewRateLimiter(8.0);
    private final SlewRateLimiter yRateLimiter = new SlewRateLimiter(8.0);
    private final SlewRateLimiter yawRateLimiter = new SlewRateLimiter(10.0);

    // Position tolerance thresholds
    private static final double X_TOLERANCE = 0.15; // meters
    private static final double Y_TOLERANCE = 0.15; // meters
    private static final double YAW_TOLERANCE = 2 * Math.PI / 180; // radians

    // Maximum output valuess
    private static final double MAX_LINEAR_SPEED = 1.4;
    private static final double MAX_ANGULAR_SPEED = 0.5;

    private static final double X_SPEED_MODIFIER = 0.5;
    private static final double Y_SPEED_MODIFIER = 0.5;
    private static final double THETA_SPEED_MODIFIER = 0.1;

    // Minimum output to overcome static friction
    private static final double MIN_LINEAR_COMMAND = 0.17;
    private static final double MIN_ANGULAR_COMMAND = 0.25;

    // State tracking
    private int framesAtTarget = 0;
    private static final int REQUIRED_FRAMES_AT_TARGET = 25;
    private int framesWithoutTarget = 0;
    private static final int MAX_FRAMES_WITHOUT_TARGET = 10;

    private final CommandSwerveDrivetrain drivetrain;
    private final int tagId;
    private final double x_offset;
    private final double y_offset;
    private final double yaw_offset;

    boolean finishedOverride;

    public ExactAlign(CommandSwerveDrivetrain drivetrain, TagRelativePose tagRelativePose) {

        finishedOverride = false;

        this.drivetrain = drivetrain;
        this.tagId = tagRelativePose.getTagId();
        this.x_offset = tagRelativePose.getX();
        this.y_offset = tagRelativePose.getY();
        this.yaw_offset = tagRelativePose.getYaw();

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

        // addRequirements(drivetrain);
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

    private int clock = 0;

    @Override
    public void execute() {
        clock++;

        // Average pose from each camera
        double avg_x = 0;
        double avg_y = 0;
        double avg_yaw = 0;

        Pose3d pose3d = VisionSubsystem.getTagRelativeToBot(tagId);

        if (pose3d == null) {
            framesWithoutTarget++;
            if (framesWithoutTarget > MAX_FRAMES_WITHOUT_TARGET) {
                end(true);
            }
            System.out.println("EXACT ALIGN REDUCE DRIVETRAIN");
            // Maintain last movement but slowly reduce it
            drivetrain.setControl(driveRequest
                    .withVelocityX(yRateLimiter.calculate(0))
                    .withVelocityY(xRateLimiter.calculate(0))
                    .withRotationalRate(yawRateLimiter.calculate(0)));
            return;
        }
        else {
            framesWithoutTarget = 0;
            avg_x = pose3d.getX();
            avg_y = pose3d.getY();
            avg_yaw = pose3d.getRotation().getZ();

            if (clock >= 20) {
                // System.out.println("average values: " + avg_x + " " + avg_y + " " + avg_yaw);
            }
        }

        // Flip yaw to face into the target
        // avg_yaw += Math.PI;

        // Clamp yaw to +-180
        // avg_yaw = Rotation2d.fromRadians(avg_yaw).getRadians();

        // Calculate errors (target offset - current position)
        double error_x = avg_x - x_offset;
        double error_y = avg_y - y_offset;
        double error_yaw = avg_yaw - yaw_offset;

        // Normalize yaw error to -π to π range
        error_yaw = Rotation2d.fromRadians(error_yaw).getRadians();

        // Calculate PID outputs
        double dx = xController.calculate(avg_x, x_offset);
        double dy = yController.calculate(avg_y, y_offset);
        double dtheta = yawController.calculate(avg_yaw, yaw_offset);

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
        boolean thetaTollerenace = Math.abs(error_yaw) < YAW_TOLERANCE;
        if (xTollerenace)
            dx = 0;
        if (yTollerenace)
            dy = 0;
        if (thetaTollerenace)
            dtheta = 0;

        // Set the drive request

        drivetrain.setControl(driveRequest
                .withVelocityX(-dx)
                .withVelocityY(-dy)
                .withRotationalRate(dtheta)
            );

        if (clock >= 20) {
            System.out.println("EXACT ALIGN VALUES: " + (-dx) + " " + (-dy) + " " + (dtheta));
            System.out.println("EXACT ALIGN VALUES: " + xTollerenace + " " + yTollerenace + " " + thetaTollerenace);
        }

        // Update state for isFinished
        if (xTollerenace && yTollerenace && thetaTollerenace) {
            framesAtTarget++;
        } else {
            framesAtTarget = 0;
        }

        if (clock >= 20) {
            clock = 0;
        }
    }

    @Override
    public void end(boolean interrupted) {
        finishedOverride = true;
        drivetrain.setControl(new SwerveRequest.SwerveDriveBrake());
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
}