package frc.robot.seafinder2.commands;

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
        // addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        System.out.println("STATIC BACK COMAND STARTED");
        done = false;
    }

    private int clock = 0;

    @Override
    public void execute() {
        // if (!visionSubsystem.canSeeTag(tag_id)) {
        // done = true;
        // return;
        // }
       // drivetrain.setControl(new SwerveRequest.RobotCentric().withVelocityX(speed));
        drivetrain.setControl(new SwerveRequest.RobotCentric().withVelocityX(speed));
        double distance = drivetrain.getRobotX();

        if (clock >= 20) {
            System.out.println("Provided Distance:  " + providedDistance + " startingdistance " + startingDistance + " distance " + distance);
            System.out.println("delta " + (distance - startingDistance));
        }

        if (clock >= 20) {clock = 0;}
    }

    @Override
    public boolean isFinished() {
        return done;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("STATIC BACK COMMAND ENDED");
        drivetrain.setControl(new SwerveRequest.SwerveDriveBrake());
    }

}