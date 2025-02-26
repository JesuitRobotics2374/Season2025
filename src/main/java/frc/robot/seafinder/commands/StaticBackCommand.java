package frc.robot.seafinder.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;

public class StaticBackCommand extends Command {

    private final CommandSwerveDrivetrain drivetrain;
    private double time;
    private double speed;
    private WaitCommand waitCommand = null;

    public StaticBackCommand(CommandSwerveDrivetrain drivetrain, double time, double speed) {
        this.drivetrain = drivetrain;
        this.time = time;
        this.speed = speed;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        System.out.println("Static X init");
    }

    @Override
    public void execute() {
        if (waitCommand == null) {
            waitCommand = new WaitCommand(time);
            waitCommand.schedule();
        }

        drivetrain.setControl(new SwerveRequest.RobotCentric().withVelocityX(speed));
    }

    @Override
    public boolean isFinished() {
        if (waitCommand != null) {
            return waitCommand.isFinished();
        } else {
            return false;
        }
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Static X complete!");
        drivetrain.setControl(new SwerveRequest.SwerveDriveBrake());
    }

}