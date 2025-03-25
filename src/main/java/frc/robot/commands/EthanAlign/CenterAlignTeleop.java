//Juli's take
//Note to Ethan; your google doc made no sense to my brain, so i just tried to write what i think you meant.

package frc.robot.commands.EthanAlign;

import org.photonvision.PhotonCamera;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.TunerConstants;

public class CenterAlignTeleop extends Command {

    private final CommandSwerveDrivetrain drivetrain;
    private final VisionSubsystem visionSubsystem;
    private Pose2d robotRelativeTagPose;

    private double tagAlignAngle; // in degrees
    private double turnSpeed = 90; // In DEGREES / SEC
    private double turnSpeedScalar = 1; //Scales the turnspeed down according to how far we are from alignment

    private double distanceFromTagAlignY; //In meters
    private double moveSpeedY = 0.5; //in meters per second
    private double moveSpeedScalar = 1; //Scales the movespeed down according to how far we are from the tag
    private double distanceFromTagAlignX; //In meters
    private double moveSpeedX = 0.5; //in meters per second
                
    public CenterAlignTeleop(CommandSwerveDrivetrain drivetrain, VisionSubsystem visionSubsystem, Pose2d robotRelativeTagPose) {
                
        this.drivetrain = drivetrain;
        this.visionSubsystem = visionSubsystem;
        this.robotRelativeTagPose = robotRelativeTagPose;

        addRequirements(drivetrain); // Require the drivetrain subsystem
        System.out.println("CAT Instantiated");
    }

    @Override
    public void initialize() {

        //ONLY WORKS IN ONE CASE SO FAR, WILL WORK ON MORE AFTER TESTING
        //theoretically should work for all cases unless i'm forgetting something

        double x = robotRelativeTagPose.getX();
        double y = robotRelativeTagPose.getY();
        double yaw = robotRelativeTagPose.getRotation().getDegrees();
        double a = Math.asin(y / x);

        double rotate = a + yaw;
        tagAlignAngle = rotate;

        distanceFromTagAlignY = y;
        distanceFromTagAlignX = x;      //may have to subtract a bit so the bot doesn't crash into apriltag

        turnSpeed = moveSpeedY * rotate / y;
        moveSpeedX = moveSpeedY * x / Math.abs(y);

        if (robotRelativeTagPose.getY() > 0) {
            turnSpeed *= -1;
            moveSpeedY *= 1;
        }
        else {
            turnSpeed *= 1;
            moveSpeedY *= -1;
        }

        System.out.println("CAT initialized");
    }

    @Override
    public void execute() {
        drivetrain.setControl(new SwerveRequest.RobotCentric().withRotationalRate(turnSpeed * turnSpeedScalar)
                                                              .withVelocityY(moveSpeedY * moveSpeedScalar)
                                                              .withVelocityX(moveSpeedX * moveSpeedScalar));

        tagAlignAngle = tagAlignAngle + (turnSpeed * turnSpeedScalar * 0.02); // Subtracts how far we've turned per seconds every 0.02 seconds (20ms, the periodic time)
        distanceFromTagAlignY = distanceFromTagAlignY + (moveSpeedY * moveSpeedScalar * 0.02);
        distanceFromTagAlignX = distanceFromTagAlignX + (moveSpeedX * moveSpeedScalar * 0.02);

        System.out.println("Center Align Executed");
    }

    @Override
    public boolean isFinished() {
        return Math.abs(distanceFromTagAlignY) < 0.1 && Math.abs(tagAlignAngle) < 0.1 && Math.abs(distanceFromTagAlignX) < 0.1;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("CAT complete!");
        drivetrain.setControl(new SwerveRequest.SwerveDriveBrake());
    }

}