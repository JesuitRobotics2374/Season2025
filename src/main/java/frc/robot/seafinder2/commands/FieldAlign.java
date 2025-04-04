package frc.robot.seafinder2.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.seafinder2.SF2Constants;
import frc.robot.seafinder2.utils.Target.TagRelativePose;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;

public class FieldAlign extends Command {

    private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric()
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
    private static final double YAW_TOLERANCE = 3 * Math.PI / 180; // radians
    
    // Maximum output values
    private static final double MAX_LINEAR_SPEED = 0.8;
    private static final double MAX_ANGULAR_SPEED = 100.0;
    
    // Minimum output to overcome static friction
    private static final double MIN_LINEAR_COMMAND = 0.05;
    private static final double MIN_ANGULAR_COMMAND = 0.17;
    
    // State tracking
    private int framesAtTarget = 0;
    private static final int REQUIRED_FRAMES_AT_TARGET = 5;
    private int framesWithoutTarget = 0;
    private static final int MAX_FRAMES_WITHOUT_TARGET = 10;

    private final CommandSwerveDrivetrain drivetrain;
    private final int tagId;
    private final double x_offset;
    private final double y_offset;
    private final double yaw_offset;

    boolean finishedOverride;

    public FieldAlign(CommandSwerveDrivetrain drivetrain, int tagId, double x, double y, double yaw) {

        finishedOverride = false;

        this.drivetrain = drivetrain;
        this.tagId = tagId;
        this.x_offset = x;
        this.y_offset = y;
        this.yaw_offset = yaw;
        
        // Initialize PID controllers
        // X PID coefficients (Adjust these values based on testing)
        xController = new PIDController(1.6, 0.0, 1.2);
        xController.setTolerance(0.3);
        
        // Y PID coefficients
        yController = new PIDController(2.6, 0.0, 0.4);
        yController.setTolerance(0.3);
        
        // Yaw PID coefficients
        yawController = new PIDController(1.6, 0.0, 0.0);
        yawController.setTolerance(0.1);
        yawController.enableContinuousInput(-Math.PI, Math.PI);
        
        // addRequirements(drivetrain);
    }

    @Override
    public void initialize() {

        finishedOverride = false;

        System.out.println("FIELDALIGN STARTED");
        
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

        Pose2d robotPose = drivetrain.getEstimator();

        // Calculate errors (target offset - current position)
        double error_x = robotPose.getX() - x_offset;
        double error_y = robotPose.getY() - y_offset;
        double error_yaw = robotPose.getRotation().getRadians() - yaw_offset;
        
        // Normalize yaw error to -π to π range
        error_yaw = Rotation2d.fromRadians(error_yaw).getRadians();

        // Calculate PID outputs
        double dx = xController.calculate(robotPose.getX(), x_offset);
        double dy = yController.calculate(robotPose.getY(), y_offset);
        double dtheta = yawController.calculate(robotPose.getRotation().getRadians(), yaw_offset);
        
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
        dx = Math.max(-MAX_LINEAR_SPEED, Math.min(dx, MAX_LINEAR_SPEED));
        dy = Math.max(-MAX_LINEAR_SPEED, Math.min(dy, MAX_LINEAR_SPEED));
        dtheta = Math.max(-MAX_ANGULAR_SPEED, Math.min(dtheta, MAX_ANGULAR_SPEED));
        
        // Apply rate limiting for smoother motion
        dx = xRateLimiter.calculate(dx);
        dy = yRateLimiter.calculate(dy);
        dtheta = yawRateLimiter.calculate(dtheta);
        
        // Zero out commands if we're within tolerance
        boolean xTollerenace = Math.abs(error_x) < 1;
        boolean yTollerenace = Math.abs(error_y) < 1;
        boolean thetaTollerenace = Math.abs(error_yaw) < 0.2;
        if (xTollerenace) dx = 0;
        if (yTollerenace) dy = 0;
        if (thetaTollerenace) dtheta = 0;

        // Set the drive request
        drivetrain.setControl(driveRequest
                .withVelocityX(dx)
                .withVelocityY(dy)
                .withRotationalRate(dtheta)
        );

        if (clock >= 20) {
            System.out.println("EXACT ALIGN VALUES: " + error_x + " " + error_y + " " + error_yaw);
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
            System.out.println("FIELDALIGN INTERRUPTED");
        } else {
            System.out.println("FIELDALIGN FINISHED");
        }
    }

    @Override
    public boolean isFinished() {
        return framesAtTarget >= REQUIRED_FRAMES_AT_TARGET || finishedOverride;
    }
}