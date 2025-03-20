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
    private double tagAlignAngle; // in degrees
    private double turnSpeed = 0.5; // change this later, possibly make dynamic or keep constant, in RADIANS / SEC

    public TeleopRotate(CommandSwerveDrivetrain drivetrain, VisionSubsystem visionSubsystem, int tag_id) {
        this.drivetrain = drivetrain;
        this.visionSubsystem = visionSubsystem;

        addRequirements(drivetrain); // Require the drivetrain subsystem
    }

    @Override
    public void initialize() {
        if (visionSubsystem.canSeeTag()) {
            Pose2d aprilTagPose = visionSubsystem.aprilTagFieldLayout.getTagPose(visionSubsystem.getTagID()).get().toPose2d();
            tagAlignAngle = aprilTagPose.getRotation().getDegrees();
            if (tagAlignAngle < 0) {
                turnSpeed *= -1;
            }
        } else
            cancel(); // Check if this works
    }

    @Override
    public void execute() {
        if (visionSubsystem.canSeeTag()) {
            Pose2d aprilTagPose = visionSubsystem.aprilTagFieldLayout.getTagPose(visionSubsystem.getTagID()).get().toPose2d();
            tagAlignAngle =aprilTagPose.getRotation().getDegrees(); // in degrees
            drivetrain.setControl(new SwerveRequest.RobotCentric().withRotationalRate(turnSpeed));
        }
    }

    @Override
    public boolean isFinished() {
        return tagAlignAngle > -5.7 && tagAlignAngle < 5.7; // 0.1 radians in degrees
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Rotation Complete!");
        // Stop the drivetrain when the command ends
        drivetrain.setControl(new SwerveRequest.RobotCentric().withRotationalRate(0));
        ;
    }

}