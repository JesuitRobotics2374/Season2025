package frc.robot.commands.teleop;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;

/**
 * DriveDynamic - Moves the robot forward by a specified distance.
 */
public class TeleopRotate extends InstantCommand {

    private CommandSwerveDrivetrain drivetrain;
    private ProfiledPIDController controller;
    private VisionSubsystem visionSubsystem;

    private int tag_id;
    private double radiansFromTag;

    public TeleopRotate(CommandSwerveDrivetrain drivetrain, VisionSubsystem visionSubsystem, int tag_id) {
        this.drivetrain = drivetrain;
        this.visionSubsystem = visionSubsystem;
        this.tag_id = tag_id;

        controller = new ProfiledPIDController(0.4, 0.4, 0.4, null);

        addRequirements(drivetrain); // Require the drivetrain subsystem
    }

    @Override
    public void execute() {
        radiansFromTag = visionSubsystem.getTagPose3d(tag_id).getRotation().getAngle();

        double rotateSpeed = controller.calculate(radiansFromTag);

        drivetrain.setControl(new SwerveRequest.RobotCentric().withRotationalRate(rotateSpeed));
    }

    @Override
    public boolean isFinished() {
        return Math.abs(radiansFromTag) < 0.15;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Movement X complete!");
        // Stop the drivetrain when the command ends
        drivetrain.setControl(new SwerveRequest.RobotCentric().withVelocityX(0));
    }
}