package frc.robot.commands.EthanAlign;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.EthanAlign.TeleopMoveX;
import frc.robot.commands.EthanAlign.TeleopMoveY;
import frc.robot.commands.EthanAlign.TeleopRotate;
import frc.robot.commands.auto.Outtake;
import frc.robot.subsystems.OuttakeSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;

public class ApproachTagTeleop extends InstantCommand {

    public ApproachTagTeleop(CommandSwerveDrivetrain drivetrain, VisionSubsystem visionSubsystem, OuttakeSubsystem outtakeSubsystem) {

        Pose2d robotRelativeTagPose = visionSubsystem.getRobotRelativeTagPose();

        Command rotate = new TeleopRotate(drivetrain, visionSubsystem, robotRelativeTagPose);
        Command moveY = new TeleopMoveY(drivetrain, visionSubsystem, robotRelativeTagPose);
        Command moveX = new TeleopMoveX(drivetrain, visionSubsystem, robotRelativeTagPose);

        Command timer = new WaitCommand(0.6);
        Command outtake = outtakeSubsystem.runOnce(() -> outtakeSubsystem.outtake());
        Command stopOuttake = outtakeSubsystem.runOnce(() -> outtakeSubsystem.stopIntake());

        SequentialCommandGroup outtakeSequence = new SequentialCommandGroup(outtake, timer, stopOuttake);


        SequentialCommandGroup approach = new SequentialCommandGroup(rotate, moveY, moveX).andThen(outtakeSequence);

        approach.schedule();
    }
}