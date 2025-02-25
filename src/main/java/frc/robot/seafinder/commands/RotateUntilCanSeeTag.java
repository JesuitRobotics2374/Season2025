package frc.robot.seafinder.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;

public class RotateUntilCanSeeTag extends Command {

    private final CommandSwerveDrivetrain drivetrain;

    private boolean done;

    public RotateUntilCanSeeTag(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        System.out.println("Rotate until can see tag initialized");
        done = false;
    }

    @Override
    public void execute() {
        drivetrain.setControl(
                new SwerveRequest.FieldCentric().withRotationalRate(Math.PI / 3.5).withVelocityX(-0.3));

        if (NetworkTableInstance.getDefault().getTable("limelight-left").getEntry("tid").getDouble(-1) != -1
        || NetworkTableInstance.getDefault().getTable("limelight-right").getEntry("tid").getDouble(-1) != -1) {
            done = true;
        }

    }

    @Override
    public boolean isFinished() {
        return done;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Init Rotate complete!");
        drivetrain.setControl(new SwerveRequest.SwerveDriveBrake());
    }

}