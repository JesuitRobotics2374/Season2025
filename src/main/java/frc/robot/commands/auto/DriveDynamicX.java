package frc.robot.commands.auto;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;

public class DriveDynamicX extends Command {

    private final CommandSwerveDrivetrain drivetrain;
    private double providedDistance;
    private double speed;

    private boolean done;

    public DriveDynamicX(CommandSwerveDrivetrain drivetrain, double distance, double speed) {
        this.drivetrain = drivetrain;
        this.providedDistance = distance;
        this.speed = speed;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        done = false;
    }

    @Override
    public void execute() {
        // if (!visionSubsystem.canSeeTag(tag_id)) {
        // done = true;
        // return;
        // }
        drivetrain.setControl(new SwerveRequest.RobotCentric().withVelocityX(speed));
        double distance = drivetrain.getForwardRange();
        System.out.println(distance);
        if (speed > 0 && distance <= providedDistance) {
            done = true;
        } else if (distance >= providedDistance) {
            done = true;
        }
    }

    @Override
    public boolean isFinished() {
        return done;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Dynamic X complete!");
        drivetrain.setControl(new SwerveRequest.SwerveDriveBrake());
    }

}