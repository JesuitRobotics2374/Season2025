package frc.robot.seafinder.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;

public class StaticBackCommand extends Command {

    private final CommandSwerveDrivetrain drivetrain;
    private double providedDistance;
    private double speed;

    private double startingDistance;

    private boolean done;

    public StaticBackCommand(CommandSwerveDrivetrain drivetrain, double distance, double speed) {
        this.drivetrain = drivetrain;
        this.providedDistance = distance;
        this.speed = speed;
        this.startingDistance = drivetrain.getRobotX();
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        System.out.println("Static X init");
        done = false;
    }

    @Override
    public void execute() {
        // if (!visionSubsystem.canSeeTag(tag_id)) {
        // done = true;
        // return;
        // }
        drivetrain.setControl(new SwerveRequest.RobotCentric().withVelocityX(speed));
        double distance = drivetrain.getRobotX();
        System.out.println(distance);
        if (distance - startingDistance < providedDistance) {
            done = true;
        }
    }

    @Override
    public boolean isFinished() {
        return done;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Static X complete!");
        drivetrain.setControl(new SwerveRequest.SwerveDriveBrake());
    }

}