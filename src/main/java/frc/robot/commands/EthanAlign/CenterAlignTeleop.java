//Juli's take
//Note to Ethan; your google doc made no sense to my brain, so i just tried to write what i think you meant.

//Ethan fixes added, leaving out X for now (for simplicity until this works0

package frc.robot.commands.EthanAlign;

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
    private double turnSpeed = Math.PI / 2; // In DEGREES / SEC
    private double turnSpeedScalar = 1; //Scales the turnspeed down according to how far we are from alignment

    private double distanceFromTagAlignY; //In meters
    private double moveSpeedY = 0.5; //in meters per second
  
    private double distanceFromTagAlignX; //In meters
    private double moveSpeedX = 0.5; //in meters per second
                
    private double moveSpeedScalar = 1; //Scales the movespeed down according to how far we are from the tag

    private boolean tagPerpendicular = false;
    private boolean yDone = false;
        
    public CenterAlignTeleop(CommandSwerveDrivetrain drivetrain, VisionSubsystem visionSubsystem, Pose2d robotRelativeTagPose) {
        this.drivetrain = drivetrain;
        this.visionSubsystem = visionSubsystem;
        this.robotRelativeTagPose = robotRelativeTagPose;

        addRequirements(drivetrain); // Require the drivetrain subsystem
        System.out.println("CAT Instantiated");
    }

    @Override
    public void initialize() {

        double x = robotRelativeTagPose.getX();
        double y = robotRelativeTagPose.getY();
        double yaw = robotRelativeTagPose.getRotation().getDegrees();
        double a = Math.atan(y / x) * 180 / Math.PI;
        
        if (y < 0) {
            tagAlignAngle = a - yaw;
        }
        else {
            tagAlignAngle = a + yaw;
        }

        distanceFromTagAlignY = Math.abs(y); //degrees
        distanceFromTagAlignX = x;      //may have to subtract a bit so the bot doesn't crash into apriltag

        moveSpeedX = 0;

        if (robotRelativeTagPose.getY() < 0) {
            turnSpeed *= -1;
            moveSpeedY *= 1;
        }
        else {
            turnSpeed *= 1;
            moveSpeedY *= -1;
        }

        tagPerpendicular = tagAlignAngle < 5;
        yDone = distanceFromTagAlignY < 0.1;

        System.out.println("CAT initialized");
    }

    @Override
    public void execute() {

        if (!tagPerpendicular) {
            drivetrain.setControl(new SwerveRequest.RobotCentric().withRotationalRate(turnSpeed * turnSpeedScalar));
            
            tagAlignAngle = tagAlignAngle - Math.abs((turnSpeed * 180 / Math.PI) * turnSpeedScalar * 0.02);

            if (tagAlignAngle < 30) {
                turnSpeedScalar = 0.5;
            }

            tagPerpendicular = tagAlignAngle < 5;
        }
        else if (!yDone) {
            drivetrain.setControl(new SwerveRequest.RobotCentric().withVelocityY(moveSpeedY * moveSpeedScalar));

            if (visionSubsystem.canSeeTag()) {
                moveSpeedScalar = 0.5;
                distanceFromTagAlignY = Math.abs(visionSubsystem.getRobotRelativeTagPose().getY());
            }
            else {
                distanceFromTagAlignY = distanceFromTagAlignY - Math.abs(moveSpeedY * moveSpeedScalar * 0.02);
            }
                //distanceFromTagAlignX = distanceFromTagAlignX - (moveSpeedX * moveSpeedScalar * 0.02);

            yDone = distanceFromTagAlignY < 0.1;
        }

        System.out.println("Center Align Executed");
    }

    @Override
    public boolean isFinished() {
        return tagPerpendicular && yDone;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("CAT complete!");
        drivetrain.setControl(new SwerveRequest.SwerveDriveBrake());
    }
}
