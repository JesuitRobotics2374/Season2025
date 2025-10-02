package frc.robot.seafinder2.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.seafinder2.SF2Constants;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;

public class CanRangeStation extends Command {

    private final SwerveRequest.RobotCentric driveRequest = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final CommandSwerveDrivetrain drivetrain;

    private double distance;

    public CanRangeStation(CommandSwerveDrivetrain drivetrain){
        this.drivetrain = drivetrain;

        // addRequirements(drivetrain);
    }

    @Override
    public void initialize(){
        System.out.println("CanRangeStation Initialized");
    }

    @Override
    public void execute() {
        distance = drivetrain.getBackRange();

        if (distance > SF2Constants.CAN_RANGE_STATION_DISTANCE) {
            drivetrain.setControl(driveRequest.withVelocityX(-1.5));
        }
    }

    @Override
    public boolean isFinished() {

        return (distance <= SF2Constants.CAN_RANGE_STATION_DISTANCE);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(new SwerveRequest.SwerveDriveBrake());
        System.out.println("CanRangeDynamicForward Ended");
    }
}