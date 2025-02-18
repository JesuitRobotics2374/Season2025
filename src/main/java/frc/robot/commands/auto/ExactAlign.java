package frc.robot.commands.auto;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;

public class ExactAlign extends Command {

    private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final CommandSwerveDrivetrain drivetrain;
    private final Pose2d targetPose;

    private double relativeDistanceMeters;
    private double targetPositionMeters;

    private double targetX;
    private double targetY;
    private double targetRotation;

    private boolean doneMoving;
    private boolean doneRotating;

    public ExactAlign(CommandSwerveDrivetrain drivetrain, Pose2d targetPose)  {
        this.drivetrain = drivetrain;
        this.targetPose = targetPose;
        
        this.targetX = targetPose.getX();
        this.targetY = targetPose.getY();
        this.targetRotation = targetPose.getRotation().getRadians();

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        doneMoving = false;
        doneRotating = false;
    }

    @Override
    public void execute() {
        Translation2d robotPosition = drivetrain.getState().Pose.getTranslation();
        double robotRotation = drivetrain.getState().Pose.getRotation().getRadians();
        
        double distanceToTarget = robotPosition.getDistance(new Translation2d(targetX, targetY));
        if (distanceToTarget < Constants.GENERIC_DISTANCE_THRESHOLD) {doneMoving = true;}

        double rotationToTarget = Math.abs(robotRotation - targetRotation);
        if (rotationToTarget < Constants.GENERIC_ROTATION_THRESHOLD) {doneRotating = true;}



        double velocityX = 0;
        double velocityY = 0;
        double rotationalRate = 0;

        if (!doneMoving) {
            double deltaX = targetX - robotPosition.getX();
            double deltaY = targetY - robotPosition.getY();
            double magnitude = Math.sqrt(deltaX * deltaX + deltaY * deltaY);

            //System.out.println("DELTA: "+deltaX +", "+ deltaY + "Target: " + targetX + ", " + targetY  + "Robot: " + robotPosition.getX() + ", " + robotPosition.getY());
            velocityX = deltaX * Constants.ALIGN_MOVE_SPEED;
            velocityY = deltaY * Constants.ALIGN_MOVE_SPEED;
            System.out.println(deltaX + " " + deltaY + " " + robotPosition.getX() + " " + robotPosition.getY());

        }
        if (!doneRotating) {
            double rotationError = targetRotation - robotRotation;
            double RESign = rotationError / Math.abs(rotationError);
            rotationalRate = rotationError * Constants.ALIGN_ROTATE_SPEED
            + (RESign * Constants.ALIGN_ROTATIONAL_FEED_FORWARD);
        }

        // Use this code if there is not a problem with sending a rotation request w/ 0 velocity
        // if (!(doneMoving && doneRotating)) {
        //   drivetrain.setControl(driveRequest.withVelocityX(-velocityX).withVelocityY(-velocityY).withRotationalRate(-rotationalRate));
        // }

        if (!doneMoving) {
            drivetrain.setControl(
                    driveRequest.withVelocityX(velocityX).withVelocityY(velocityY)
                            .withRotationalRate(-rotationalRate));
        } else if (!doneRotating) {
            drivetrain.setControl(
                    driveRequest.withRotationalRate(-rotationalRate));
        }
    }

    @Override
    public boolean isFinished() {
        return doneMoving && doneRotating;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Alignment complete!");
        drivetrain.setControl(new SwerveRequest.SwerveDriveBrake());
    }

}