package frc.robot.commands.EthanAlign;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;

public class TeleopMoveY extends Command {

    private final CommandSwerveDrivetrain drivetrain;
    private final VisionSubsystem visionSubsystem;
    
    private Pose2d robotRelativeTagPose; //Tag pose relative to the bot

    private double distanceFromTagAlign; //In meters
    private double moveSpeed = 0.5;
    private double moveSpeedScalar = 1; //Scales the movespeed down according to how far we are from the tag

    public TeleopMoveY(CommandSwerveDrivetrain drivetrain, VisionSubsystem visionSubsystem, Pose2d robotRelativeTagPose) {
        this.drivetrain = drivetrain;
        this.visionSubsystem = visionSubsystem;
        this.robotRelativeTagPose = robotRelativeTagPose;

        addRequirements(drivetrain); // Require the drivetrain subsystem
        System.out.println("Y Instantiated");
    }

    @Override
    public void initialize() {
        distanceFromTagAlign = robotRelativeTagPose.getY();
        
        if (visionSubsystem.canSeeTag()) {
            Pose2d robotRelativeTagPose = visionSubsystem.getRobotRelativeTagPose();
            distanceFromTagAlign = robotRelativeTagPose.getY();

            if (distanceFromTagAlign < 0) {
                moveSpeed *= -1;
            }
        }
        System.out.println("Y Initialized");
    }

    @Override
    public void execute() {
        if (visionSubsystem.canSeeTag()) {
            Pose2d robotRelativeTagPose = visionSubsystem.getRobotRelativeTagPose();
            distanceFromTagAlign = robotRelativeTagPose.getY();
            //moveSpeedScalar = distanceFromTagAlign / 5;
        }

        drivetrain.setControl(new SwerveRequest.RobotCentric().withVelocityY(moveSpeed * moveSpeedScalar));
        System.out.println("Y Executed");
    }

    @Override
    public boolean isFinished() {
        return Math.abs(distanceFromTagAlign) < 0.1;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Movement Y complete!");
        // Stop the drivetrain when the command ends
        drivetrain.setControl(new SwerveRequest.SwerveDriveBrake());
    }
}