// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.auto.DriveDynamicX;
import frc.robot.commands.auto.StaticBackCommand;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.utils.FMapConstant;

public class PathfindCommand extends SequentialCommandGroup {

    public static boolean wasAligned = false;

    public enum Alignment {

        LEFT, RIGHT, CENTER;

        private int modif;

        static {
            LEFT.modif = -1;
            RIGHT.modif = 1;
            CENTER.modif = 0;
        }

        public int getModif() {
            return modif;
        }

        public static Alignment parseTopology(boolean isReef, int loc) {
            if (loc < 0 || loc > 7) {
                throw new IllegalArgumentException("Invalid topology");
            }
            return isReef ? ((loc % 2 == 0) ? LEFT : RIGHT) : CENTER;
        }

    }

    public PathfindCommand(CommandSwerveDrivetrain drivetrain, int tagId, Alignment alignment) {

        // int tagId = (int) LimelightHelpers.getFiducialID("limelight-left");
        // int tagId = 18;

        System.out.println(tagId);

        if (tagId == -1) {
            System.out.println("NO TAG VISIBLE");
            return;
        }

        Pose3d tagTarget = FMapConstant.getFMapPosition(tagId);

        if (tagTarget == null) {
            System.out.println("TARGET IS NULL");
            return;
        }

        System.out.println(" --- TAG DATA --- ");
        System.out.println(tagTarget.getX());
        System.out.println(tagTarget.getY());
        System.out.println(tagTarget.getZ());
        System.out.println(tagTarget.getRotation().getX());
        System.out.println(tagTarget.getRotation().getY());
        System.out.println(tagTarget.getRotation().getZ());

        Rotation3d tagRotation = tagTarget.getRotation().plus(new Rotation3d(0, 0, Math.PI));

        int modifier = alignment.getModif();

        // For some unknown reason, the modifier must be flipped for vertical faces of the reef.
        // This may be a mathematical error; this is a temporary fix.
        if (tagId == 7 || tagId == 10 || tagId == 18 || tagId == 21) {
            System.out.println("Vertical face detected; flipping modifier");
            modifier *= -1;
        }

        Pose3d target3d = new Pose3d(
                tagTarget.getX() + Constants.PATHFINDING_FRONT_BUFFER * Math.cos(tagRotation.getZ())
                        + Constants.PATHFINDING_SHIFT_FACTOR * Math.sin(tagRotation.getZ()) * modifier
                        + Constants.FIELD_X_MIDPOINT,
                tagTarget.getY() + Constants.PATHFINDING_FRONT_BUFFER * Math.sin(tagRotation.getZ())
                        + Constants.PATHFINDING_SHIFT_FACTOR * Math.cos(tagRotation.getZ()) * modifier
                        + Constants.FIELD_Y_MIDPOINT,
                tagTarget.getZ(),
                tagRotation);

        Pose2d target = target3d.toPose2d();

        PathConstraints constraints = new PathConstraints(
                Constants.PATHFINDING_MAX_VELOCITY,
                Constants.PATHFINDING_MAX_ACCELERATION,
                Constants.PATHFINDING_MAX_ROTATIONAL_VELOCITY,
                Constants.PATHFINDING_MAX_ROTATIONAL_ACCELERATION);

        System.out.println(target);

        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        Command pathfindingCommand = AutoBuilder.pathfindToPose(
                target,
                constraints,
                0);

        // pathfindingCommand.schedule();

        Command driveDynamic = new DriveDynamicX(drivetrain, 0.297, 0.2);

        System.out.println("PATHFIND TO " + target.toString() + " STARTED");
        System.out.println("Aligned Prior: " + wasAligned);

        if (wasAligned) {
            // addCommands(new StaticBackCommand(drivetrain, -0.2, -1), new WaitCommand(1), pathfindingCommand, driveDynamic);
            addCommands(new WaitCommand(1), pathfindingCommand, driveDynamic);
        } else {
            addCommands(new WaitCommand(1), pathfindingCommand, driveDynamic);
        }

        wasAligned = true;

        addRequirements(drivetrain);
    }
}
