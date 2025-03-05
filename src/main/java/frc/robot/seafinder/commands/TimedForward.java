package frc.robot.seafinder.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;

public class TimedForward extends Command {

    private final SwerveRequest.RobotCentric driveRequest = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final CommandSwerveDrivetrain drivetrain;

    private double operationClock = 0;

    public TimedForward(CommandSwerveDrivetrain drivetrain, double seconds){
        this.drivetrain = drivetrain;
        this.operationClock = seconds * 50;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize(){
        System.out.println("TimedForward Initialized");
        drivetrain.setControl(driveRequest.withVelocityX(0.5));
    }

    @Override
    public void execute() {
        operationClock--;
    }

    @Override
    public boolean isFinished() {
        return operationClock <= 0;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(new SwerveRequest.SwerveDriveBrake());
        System.out.println("TimedForward Ended");
    }
}
