// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.auto.DriveDynamicX;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.utils.FMapConstant;
import frc.robot.utils.LimelightHelpers;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class OrganizePathfind extends SequentialCommandGroup {
  /** Creates a new OrganizePathfind. */
  public OrganizePathfind(CommandSwerveDrivetrain drivetrain, int tagId) {

    // int tagId = (int) LimelightHelpers.getFiducialID("limelight-left");
    // int tagId = 18;

    System.out.println(tagId);

    if (tagId == -1) {
      System.out.println("NO TAG VISIBLE");
      return;
    }

    Pose2d target = FMapConstant.getFMapPosition(tagId).toPose2d();

    if (target == null) {
      System.out.println("TARGET IS NULL");
      return;
    }

    PathConstraints constraints = new PathConstraints(
        3.5, 1, // 3 - 4
        Units.degreesToRadians(540),
        Units.degreesToRadians(720));

    System.out.println(target);

    // Since AutoBuilder is configured, we can use it to build pathfinding commands
    Command pathfindingCommand = AutoBuilder.pathfindToPose(
        target,
        constraints,
        0);

    // pathfindingCommand.schedule();

    Command driveDynamic = new DriveDynamicX(drivetrain, 0.167, 0.2);

    System.out.println("PATHFIND TO " + target.toString() + " STARTED");

    addCommands(new WaitCommand(1), pathfindingCommand, driveDynamic);

    addRequirements(drivetrain);
  }
}
