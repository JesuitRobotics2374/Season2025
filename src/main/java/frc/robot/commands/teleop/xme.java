package frc.robot.commands.teleop;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.auto.Pathfind;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.utils.FMapConstant;
import frc.robot.utils.LimelightHelpers;

public class xme extends InstantCommand {
    private CommandSwerveDrivetrain drivetrain;

    public xme(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;

        System.out.println("pee pee poo poo");

        addRequirements(drivetrain);
    }

    @Override
    public void execute() {

        // Pose2d cameraPose2d = LimelightHelpers.getBotPose3d_TargetSpace("limelight-left").toPose2d();
        // Pose2d targetPose2d = LimelightHelpers.getBotPose3d_wpiBlue("limelight-left").toPose2d();

        System.out.println((int)LimelightHelpers.getFiducialID("limelight-left"));
        Pose2d test = FMapConstant.getFMapPosition((int)LimelightHelpers.getFiducialID("limelight-left")).toPose2d();

        System.out.println(test);

        // Pathfind pathfindCommand = new Pathfind(drivetrain, targetPose2d);
        // pathfindCommand.schedule();

        end(false);
    }
}
