package frc.robot.seafinder.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
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

    public StationAlign(CommandSwerveDrivetrain drivetrain){
        this.drivetrain = drivetrain;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize(){
    }

    @Override
    public void execute() {
        double rightRange = drivetrain.getForwardRangeRight() + Constants.SA_RIGHT_BUFFER;
        double leftRange = drivetrain.getForwardRangeLeft();
        double avgRange = (rightRange + leftRange) / 2;

        System.out.println("LEFT: " + leftRange + " RIGHT: " + rightRange + " AVG: " + avgRange);

        double velocityX = (0.3 * (avgRange - Constants.SA_TARGET_DISTANCE) / Constants.SA_TARGET_DISTANCE) + 0.1;

        // Check if we are at or past the target distance
        if (avgRange <= Constants.SA_TARGET_DISTANCE) {
            velocityX = 0;
            doneMoving = true;
        } else {
            doneMoving = false;
        }

        // Using the left and right range, we can determine the rotational rate to apply
        double rotationalRate = ((rightRange - leftRange) / 8) + 0.034;

        // Check if we are within the rotational rate threshold
        if ((Math.abs(leftRange - rightRange) < Constants.SA_ROTATIONAL_RATE_THRESHOLD) || avgRange >= Constants.SA_TARGET_DISTANCE+0.3) {
            rotationalRate = 0;
            doneRotating = true;
        } else {
            doneRotating = false;
        }

        drivetrain.setControl(driveRequest.withVelocityX(velocityX).withRotationalRate(rotationalRate));

    }

    @Override
    public boolean isFinished() {
        return doneMoving && doneRotating;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(new SwerveRequest.SwerveDriveBrake());
        System.out.println("Station Align Ended");
    }
}
