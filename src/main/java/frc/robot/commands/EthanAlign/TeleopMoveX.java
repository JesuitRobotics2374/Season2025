package frc.robot.commands.EthanAlign;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.TunerConstants;

public class TeleopMoveX extends Command {

    public final CommandSwerveDrivetrain drivetrain;
    private final VisionSubsystem visionSubsystem;

    private Pose2d robotRelativeTagPose; //Tag pose relative to the bot

    private double distanceFromTag; //In meters
    private double moveSpeed = 2;
    private double moveSpeedScalar = 1; //Scales the movespeed down according to how far we are from the tag
    

    public TeleopMoveX(CommandSwerveDrivetrain drivetrain, VisionSubsystem visionSubsystem, Pose2d robotRelativeTagPose) {
        this.drivetrain = drivetrain;
        this.visionSubsystem = visionSubsystem;
        this.robotRelativeTagPose = robotRelativeTagPose;

        addRequirements(drivetrain); // Require the drivetrain subsystem
        System.out.println("X Instantiated");
    }

    @Override
    public void initialize() {
       distanceFromTag = visionSubsystem.getDistanceToAprilTag();

       System.out.println(distanceFromTag);
       System.out.println("X Initialized");
    }

    @Override
    public void execute() {
        distanceFromTag = visionSubsystem.getDistanceToAprilTag();

        if (distanceFromTag < 0.5) {
            moveSpeedScalar = Math.abs(2 * distanceFromTag);
        }

        double applySpeed = moveSpeed * moveSpeedScalar;

        if (applySpeed < 0 || applySpeed > 2) {
            applySpeed = 1;
        }

        drivetrain.setControl(new SwerveRequest.RobotCentric().withVelocityX(applySpeed));
    }

    @Override
    public boolean isFinished() {
        return distanceFromTag < 0.1;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Movement X complete!");
        // Stop the drivetrain when the command ends
        drivetrain.setControl(new SwerveRequest.SwerveDriveBrake());
    }
}