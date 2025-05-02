package frc.robot.seafinder2.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;

public class TestCommand extends Command {
    CommandSwerveDrivetrain drivetrain;
    private final SwerveRequest.RobotCentric driveRequest = new SwerveRequest.RobotCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    int clock = 0;

    public TestCommand(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        clock = 0;
    }

    public void initialize() {
        clock = 0;
    }

    public void execute() {
        drivetrain.setControl(driveRequest.withVelocityX(1));
        clock++;
    }

    public boolean isFinished() {
        return clock > 50;
    }

    public void end(boolean interrupted) {
        drivetrain.setControl(new SwerveRequest.SwerveDriveBrake());
    }
}
