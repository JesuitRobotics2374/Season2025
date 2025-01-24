package frc.robot.commands.teleop;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.auto.Pathfind;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.utils.LimelightHelpers;

public class StartPathfind extends InstantCommand {
    private CommandSwerveDrivetrain drivetrain;

    public StartPathfind(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    @Override
    public void execute() {
        Pose2d targetPose2d = LimelightHelpers.getTargetPose3d_RobotSpace("limelight-left").toPose2d();

        Pathfind pathfindCommand = new Pathfind(drivetrain, targetPose2d);
        pathfindCommand.schedule();

        end(false);
    }
}
