
package frc.robot.commands.EthanAlign;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.TunerConstants;

public class SpinTest extends Command {

    private final CommandSwerveDrivetrain drivetrain;
    private final VisionSubsystem visionSubsystem;
    private Pose2d robotRelativeTagPose;

    private double turnSpeed = 0.5 * Math.PI; // In DEGREES / SEC
    //private double turnSpeedScalar = 1; //Scales the turnspeed down according to how far we are from alignment

    private int count;
    private long start;
        
    public SpinTest(CommandSwerveDrivetrain drivetrain, VisionSubsystem visionSubsystem, Pose2d robotRelativeTagPose) {
                
        this.drivetrain = drivetrain;
        this.visionSubsystem = visionSubsystem;
        this.robotRelativeTagPose = robotRelativeTagPose;

        addRequirements(drivetrain); // Require the drivetrain subsystem
    }

    @Override
    public void initialize() {
        count = 0;
        start = System.currentTimeMillis();
    }

    @Override
    public void execute() {
        drivetrain.setControl(new SwerveRequest.RobotCentric().withRotationalRate(turnSpeed));
            
        count++;
    }

    @Override
    public boolean isFinished() {
        return count == 400;
    }

    @Override
    public void end(boolean interrupted) {
        long endTime = System.currentTimeMillis();
        System.out.println("SpinTest Time: " + (endTime - start) / 1_000.0);
        drivetrain.setControl(new SwerveRequest.SwerveDriveBrake());
    }
}
