package frc.robot.seafinder.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;

public class CanRangeDynamicForward extends Command{

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
        System.out.println("Range " + drivetrain.getForwardRange());
        if (drivetrain.getForwardRange() > Constants.DYNAMIC_FORWARD_DISTANCE) {
            drivetrain.setControl(driveRequest.withVelocityY(0.5));
        }
    }

    @Override
    public boolean isFinished() {
        return (drivetrain.getForwardRange() <= Constants.DYNAMIC_FORWARD_DISTANCE);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(driveRequest.withVelocityY(0));
        System.out.println("CanRangeDynamicForward Ended");
    }
}
