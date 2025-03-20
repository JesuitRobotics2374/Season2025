package frc.robot.commands.EthanAlign;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;

public class TeleopRotate extends Command {

    private final CommandSwerveDrivetrain drivetrain;
    private final VisionSubsystem visionSubsystem;

    private Pose2d robotRelativeTagPose;
    
    private double tagAlignAngle; // in degrees
    private double turnSpeed = 90; // In DEGREES / SEC
    private double turnSpeedScalar = 1; //Scales the turnspeed down according to how far we are from alignment

    public TeleopRotate(CommandSwerveDrivetrain drivetrain, VisionSubsystem visionSubsystem, Pose2d robotRelativeTagPose) {
        this.drivetrain = drivetrain;
        this.visionSubsystem = visionSubsystem;
        this.robotRelativeTagPose = robotRelativeTagPose;

        addRequirements(drivetrain); // Require the drivetrain subsystem
    }

    @Override
    public void initialize() {
        if (visionSubsystem.canSeeTag()) {
            double theta = robotRelativeTagPose.getRotation().getDegrees();
            double beta = Math.asin(Math.abs(robotRelativeTagPose.getY()) / visionSubsystem.getDistanceToAprilTag());

            double delta = Math.abs(theta) + Math.abs(beta);

            tagAlignAngle = delta;

            if (robotRelativeTagPose.getY() > 0) {
                turnSpeed *= -1;
            }
        } else
            cancel(); // Check if this works
    }

    @Override
    public void execute() {
        drivetrain.setControl(new SwerveRequest.RobotCentric().withRotationalRate(turnSpeed * turnSpeedScalar));

        tagAlignAngle = tagAlignAngle - (Math.abs(turnSpeed) * turnSpeedScalar * 0.02); // Subtracts how far we've turned per seconds every 0.02 seconds (20ms, the periodic time)

        turnSpeedScalar = tagAlignAngle / 2;
    }

    @Override
    public boolean isFinished() {
        return tagAlignAngle < 1; // in degrees
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Rotation Complete!");
        // Stop the drivetrain when the command ends
        drivetrain.setControl(new SwerveRequest.SwerveDriveBrake());
        ;
    }

}