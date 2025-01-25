package frc.robot.commands.teleop;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.LimelightHelpers.RawFiducial;

public class ExecuteDynamic extends InstantCommand {

    CommandSwerveDrivetrain drivetrain;
    VisionSubsystem visionSubsystem;

    public ExecuteDynamic(CommandSwerveDrivetrain drivetrain, VisionSubsystem visionSubsystem) {
        this.drivetrain = drivetrain;
        this.visionSubsystem = visionSubsystem;
    }
    
    public void execute() {
        LimelightHelpers.RawFiducial closestFiducial = new LimelightHelpers.RawFiducial(-1, 0, 0, 0, Integer.MAX_VALUE, 0, 0);

        LimelightHelpers.RawFiducial[] fiducials = LimelightHelpers.getRawFiducials("limelight-left");

        for (LimelightHelpers.RawFiducial fiducial : fiducials) {
            boolean idCondition = (6 <= fiducial.id && fiducial.id <= 12) || (17 <= fiducial.id && fiducial.id <= 22);
            boolean distanceCondition = (fiducial.distToCamera < closestFiducial.distToCamera);

            if (idCondition && distanceCondition) {
                closestFiducial = fiducial;
            }
        }

        if (closestFiducial.id != -1) {
            TeleopRotate rotate = new TeleopRotate(drivetrain, visionSubsystem, closestFiducial.id);
            TeleopMoveX moveX = new TeleopMoveX(drivetrain, visionSubsystem, closestFiducial.id);
            TeleopMoveY moveY = new TeleopMoveY(drivetrain, visionSubsystem, closestFiducial.id);
            SequentialCommandGroup seqCommand = new SequentialCommandGroup(rotate, moveX, moveY);
    
            seqCommand.schedule();
            System.out.println("Dynamic command executed");
        } else {
            System.out.println("No tag detected");
        }

        end(false);
    }

    public double distance(Pose3d a) {
        return Math.pow(a.getX(), 2) + Math.pow(a.getY(), 2);
    }
}
