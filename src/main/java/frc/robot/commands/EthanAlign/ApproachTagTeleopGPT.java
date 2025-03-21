package frc.robot.commands.EthanAlign;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.EthanAlign.TeleopMoveX;
import frc.robot.subsystems.OuttakeSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;

public class ApproachTagTeleopGPT extends InstantCommand {
    private final CommandSwerveDrivetrain drivetrain;
    private final VisionSubsystem visionSubsystem;
    private final OuttakeSubsystem outtakeSubsystem;

    public ApproachTagTeleopGPT(CommandSwerveDrivetrain drivetrain, VisionSubsystem visionSubsystem, OuttakeSubsystem outtakeSubsystem) {
        this.drivetrain = drivetrain;
        this.visionSubsystem = visionSubsystem;
        this.outtakeSubsystem = outtakeSubsystem;
    }

    @Override
    public void initialize() {
        Pose2d robotRelativeTagPose = visionSubsystem.getRobotRelativeTagPose();

        Command moveX = new TeleopMoveX(drivetrain, visionSubsystem, robotRelativeTagPose);
        Command timer = new WaitCommand(0.6);
        Command outtake = outtakeSubsystem.runOnce(outtakeSubsystem::outtake);
        Command stopOuttake = outtakeSubsystem.runOnce(outtakeSubsystem::stopIntake);

        SequentialCommandGroup outtakeSequence = new SequentialCommandGroup(outtake, timer, stopOuttake);

        SequentialCommandGroup approach = new SequentialCommandGroup(
            moveX,
            timer
        ).andThen(outtakeSequence);

        // **Manually schedule the SequentialCommandGroup**
        approach.schedule();
    }
}
