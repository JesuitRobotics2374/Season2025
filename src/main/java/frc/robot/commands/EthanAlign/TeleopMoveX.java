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
import frc.robot.subsystems.drivetrain.TunerConstants;

public class TeleopMoveX extends Command {

    public final CommandSwerveDrivetrain drivetrain;
    private final VisionSubsystem visionSubsystem;

    private Pose2d robotRelativeTagPose; //Tag pose relative to the bot

    private double distanceFromTag; //In meters
    private double moveSpeed = 0.5;
    private double moveSpeedScalar = 1; //Scales the movespeed down according to how far we are from the tag
    

    public TeleopMoveX(CommandSwerveDrivetrain drivetrain, VisionSubsystem visionSubsystem, Pose2d robotRelativeTagPose) {
        this.drivetrain = drivetrain;
        this.visionSubsystem = visionSubsystem;
        this.robotRelativeTagPose = robotRelativeTagPose;

        addRequirements(drivetrain); // Require the drivetrain subsystem
    }

    @Override
    public void initialize() {
       distanceFromTag = robotRelativeTagPose.getX();
    }

    @Override
    public void execute() {
        distanceFromTag = visionSubsystem.getRobotRelativeTagPose().getX();
        moveSpeedScalar = distanceFromTag / 5;

        drivetrain.setControl(new SwerveRequest.RobotCentric().withVelocityX(moveSpeed * moveSpeedScalar));
    }

    @Override
    public boolean isFinished() {
        return distanceFromTag < 0.5;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Movement X complete!");
        // Stop the drivetrain when the command ends
        drivetrain.setControl(new SwerveRequest.SwerveDriveBrake());
    }

    //Step 1: Align perpendicularly to the tag
    //Step 2: Move sideways until we are aligned to the tag
    //Step 3: Move forward
}