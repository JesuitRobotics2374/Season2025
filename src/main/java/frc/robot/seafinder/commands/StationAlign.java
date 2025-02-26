package frc.robot.seafinder.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import java.util.ArrayDeque;
import java.util.Queue;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;

public class StationAlign extends Command{

    private final SwerveRequest.RobotCentric driveRequest = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final CommandSwerveDrivetrain drivetrain;

    private boolean doneMoving = false;
    private boolean doneRotating = false;

    private Queue<Double> avgRightRange = new ArrayDeque<Double>();
    private Queue<Double> avgLeftRange = new ArrayDeque<Double>();
    private int numSamples = 5;

    public StationAlign(CommandSwerveDrivetrain drivetrain){
        this.drivetrain = drivetrain;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize(){
        System.out.println("Station Align Initialized");
    }

    @Override
    public void execute() {
        double rightRange = drivetrain.getForwardRangeRight() + Constants.RIGHT_CAN_RANGE_OFFSET;
        double leftRange = drivetrain.getForwardRangeLeft();
        
        if (avgLeftRange.size() >= numSamples) {
            avgLeftRange.poll();
        }
        avgLeftRange.add(leftRange);

        if (avgRightRange.size() >= numSamples) {
            avgRightRange.poll();
        }
        avgRightRange.add(rightRange);


        double leftSum = 0;
        for (double range : avgLeftRange) {
            leftSum += range;
        }
        double leftAvg = leftSum / (avgLeftRange.size() + 1e-6);
        
        double rightSum = 0;
        for (double range : avgRightRange) {
            rightSum += range;
        }
        double rightAvg = rightSum / (avgRightRange.size() + 1e-6);

        double avgRange = (leftAvg + rightAvg) / 2;

        System.out.println("LEFT: " + leftRange + " RIGHT: " + rightRange + " AVE: " + avgRange);

        double velocityX = (0.25 * (avgRange - Constants.STATION_TARGET_DISTANCE) / Constants.STATION_TARGET_DISTANCE) + 0.35;

        // Check if we are at or past the target distance
        if (avgRange <= Constants.STATION_TARGET_DISTANCE) {
            velocityX = 0;
            doneMoving = true;
        }

        // Using the left and right range, we can determine the rotational rate to apply
        double rotationalRate = ((rightAvg - leftAvg) / 8) + 0.047;

        // Check if we are within the rotational rate threshold
        if ((Math.abs(leftRange - rightRange) < Constants.STATION_ROTATIONAL_RATE_THRESHOLD) || avgRange >= Constants.STATION_TARGET_DISTANCE+0.3) {
            rotationalRate = 0;
            doneRotating = true;
        }

        drivetrain.setControl(driveRequest.withVelocityX(velocityX).withRotationalRate(rotationalRate));

    }

    @Override
    public boolean isFinished() {
        System.out.println("DONE MOVING: " + doneMoving + " DONE ROTATING: " + doneRotating);
        return doneMoving && doneRotating;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(new SwerveRequest.SwerveDriveBrake());
        System.out.println("Station Align Ended");
    }
}
