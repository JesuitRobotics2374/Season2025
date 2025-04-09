package frc.robot.seafinder2.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;

public class StaticBack extends Command {

    private final SwerveRequest.RobotCentric driveRequest = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final CommandSwerveDrivetrain drivetrain;

    public StaticBack(CommandSwerveDrivetrain drivetrain){
        this.drivetrain = drivetrain;

        // addRequirements(drivetrain);
    }

    @Override
    public void initialize(){
        System.out.println("StaticBack Initialized");

        drivetrain.setControl(driveRequest.withVelocityX(-1));
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(new SwerveRequest.SwerveDriveBrake());
        System.out.println("StaticBack Ended");
    }
}