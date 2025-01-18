package frc.robot.commandsTeleop;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;

public class ForwardAlign extends InstantCommand {

    private final double moveSpeed = 0.1;
    private CommandSwerveDrivetrain drivetrain;
    private Pose2d target;

    public ForwardAlign(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    @Override

    public void initialize() {
        target = new Pose2d();
        drivetrain.setControl(new SwerveRequest.RobotCentric().withVelocityY(moveSpeed));
        System.out.println("Initialized");
    }

    @Override

    public void execute() {
        target = new Pose2d();
        if (target.getMeasureY().magnitude() > -1) {
            if (target.getMeasureY().magnitude() < 0.5) {
                drivetrain.setControl(new SwerveRequest.RobotCentric().withVelocityY(0));
                drivetrain.setControl(new SwerveRequest.RobotCentric().withVelocityX(moveSpeed));
                System.out.println("moving forwards");
            }
        }
        else System.out.println("target null");
    }

    @Override

    public void end(boolean interrupted) {
        if (target.getMeasureX().magnitude() < 1) {
            drivetrain.setControl(new SwerveRequest.SwerveDriveBrake());
            System.out.println("stopped");
        }
    }

    // private double convertToDouble(Distance distance) {
    //     String str = "" + distance.magnitude();
    //     distance.magnitude();
    //     double convert = Double.parseDouble(str);

    //     return convert;
    // }
}
