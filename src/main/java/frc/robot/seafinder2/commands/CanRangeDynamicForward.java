package frc.robot.seafinder2.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.seafinder2.SF2Constants;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;

public class CanRangeDynamicForward extends Command {

    private final SwerveRequest.RobotCentric driveRequest = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final CommandSwerveDrivetrain drivetrain;

    public CanRangeDynamicForward(CommandSwerveDrivetrain drivetrain){
        this.drivetrain = drivetrain;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize(){
        System.out.println("CanRangeDynamicForward Initialized");
    }

    @Override
    public void execute() {
        double distance = Math.min(drivetrain.getForwardRangeLeft(), drivetrain.getForwardRangeRight());

        if (distance > SF2Constants.CAN_RANGE_FORWARD_DISTANCE) {
            drivetrain.setControl(driveRequest.withVelocityX(0.5));
        }
    }

    @Override
    public boolean isFinished() {
        double distance = Math.min(drivetrain.getForwardRangeLeft(), drivetrain.getForwardRangeRight());

        return (distance <= SF2Constants.CAN_RANGE_FORWARD_DISTANCE);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(new SwerveRequest.SwerveDriveBrake());
        System.out.println("CanRangeDynamicForward Ended");
    }
}