// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.digital;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Core;
import frc.robot.commands.auto.DriveDynamicX;
import frc.robot.commands.auto.ExactAlign;
import frc.robot.commands.auto.StaticBackCommand;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.utils.FMapConstant;
import frc.robot.utils.Setpoint;

public class PathfinderSubsystem {

    private Core core; // Keep an instance of core for moveToSetpoint and performRetract
    private CommandSwerveDrivetrain drivetrain;

    private boolean locationLoaded = false; // Have we inputted a location?
    private int tagId; // The tag ID to navigate to
    private Alignment alignment; // The alignment of the robot to the tag (left/right)

    private boolean heightLoaded = false; // Have we inputted a height?
    private Setpoint reefHeight; // What setpoint should we move to? (T1/2/3/4)

    private Command runningCommand; // Keep track of the currently running command so we can override it later

    // Alignment data structure
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

    // Subsystem is now constructed just once, at the top of Core with other subsystems
    public PathfinderSubsystem(Core core) {
        this.core = core;
        this.drivetrain = core.getDrivetrain();
    }

    // Queue a pathfind; done by clicking a button on the panel
    public void queueFind(int tagId, Alignment alignment) {
        this.tagId = tagId;
        this.alignment = alignment;
        this.locationLoaded = true;
        if (heightLoaded) {
            executeSequence();
            locationLoaded = false;
            heightLoaded = false;
        } else {
            updateGUI(1);
        }
    }

    // Queue an alignment; done by clicking a button on the panel
    public void queueAlign(Setpoint reefHeight) {
        this.reefHeight = reefHeight;
        this.heightLoaded = true;
        if (locationLoaded) {
            executeSequence();
            locationLoaded = false;
            heightLoaded = false;
        } else {
            updateGUI(2);
        }
    }

    // For my GUI, you can ignore
    private void updateGUI(int d) {
        core.getNavInterfaceSubsystem().sendReadData(d);
    }

    // Unused for now, but should be implimented
    public void stopAll() {
        if (runningCommand != null) {
            runningCommand.cancel();
            runningCommand = null;
            updateGUI(99);
        }
    }

    // Do both pathfind and align; for development purposes only
    public void doAll(int tagId, Alignment alignment, Setpoint reefHeight) {
        queueFind(tagId, alignment);
        queueAlign(reefHeight);
        updateGUI(90);
    }

    // Once both pathfind and align are queued, execute the sequence
    public void executeSequence() {

        if (runningCommand != null) {
            runningCommand.cancel(); // Cancel the previously running command
        }

        System.out.println(tagId);

        if (tagId == -1) {
            System.out.println("NO TAG VISIBLE");
            updateGUI(1000);
            return;
        }

        Pose3d tagTarget = FMapConstant.getFMapPosition(tagId); // Get the tag's position from FMap

        if (tagTarget == null) {
            System.out.println("TARGET IS NULL");
            updateGUI(1000);
            return;
        }

        updateGUI(3);

        ///// PRE PATHFIND

        Rotation3d tagRotation = tagTarget.getRotation().plus(new Rotation3d(0, 0, Math.PI));

        int modifier = alignment.getModif();

        Pose3d pretarget3d = new Pose3d(
                tagTarget.getX() + Constants.PATHFINDING_PRE_BUFFER * Math.cos(tagRotation.getZ())
                        + Constants.PATHFINDING_SHIFT_FACTOR * Math.sin(tagRotation.getZ())
                                * modifier
                        + Constants.FIELD_X_MIDPOINT,
                tagTarget.getY() + Constants.PATHFINDING_PRE_BUFFER * Math.sin(tagRotation.getZ())
                        - Constants.PATHFINDING_SHIFT_FACTOR * Math.cos(tagRotation.getZ())
                                * modifier
                        + Constants.FIELD_Y_MIDPOINT,
                tagTarget.getZ(),
                tagRotation);

        Pose2d pretarget = pretarget3d.toPose2d();

        PathConstraints constraints = new PathConstraints(
                Constants.PATHFINDING_MAX_VELOCITY,
                Constants.PATHFINDING_MAX_ACCELERATION,
                Constants.PATHFINDING_MAX_ROTATIONAL_VELOCITY,
                Constants.PATHFINDING_MAX_ROTATIONAL_ACCELERATION);

        System.out.println(pretarget);
        // drivetrain.setLabel(pretarget, "pre");

        Command prepathfindingCommand = AutoBuilder.pathfindToPose(
                pretarget,
                constraints,
                0.8);

        ///// MAIN PATHFIND

        System.out.println(" --- TAG DATA --- ");
        System.out.println(tagTarget.getX());
        System.out.println(tagTarget.getY());
        System.out.println(tagTarget.getZ());
        System.out.println(tagTarget.getRotation().getX());
        System.out.println(tagTarget.getRotation().getY());
        System.out.println(tagTarget.getRotation().getZ());

        Pose3d target3d = new Pose3d(
                tagTarget.getX() + Constants.PATHFINDING_FRONT_BUFFER * Math.cos(tagRotation.getZ())
                        + Constants.PATHFINDING_SHIFT_FACTOR * Math.sin(tagRotation.getZ())
                                * modifier
                        + Constants.FIELD_X_MIDPOINT,
                tagTarget.getY() + Constants.PATHFINDING_FRONT_BUFFER * Math.sin(tagRotation.getZ())
                        - Constants.PATHFINDING_SHIFT_FACTOR * Math.cos(tagRotation.getZ())
                                * modifier
                        + Constants.FIELD_Y_MIDPOINT,
                tagTarget.getZ(),
                tagRotation);

        Pose2d target = target3d.toPose2d();

        System.out.println(target);
        // drivetrain.setLabel(target, "main");

        Command pathfindingCommand = AutoBuilder.pathfindToPose(
                target,
                constraints,
                0);

        ///// FINAL PATHFIND

        Pose3d finaltarget3d = new Pose3d(
                tagTarget.getX() + Constants.PATHFINDING_POST_BUFFER * Math.cos(tagRotation.getZ())
                        + Constants.PATHFINDING_SHIFT_FACTOR * Math.sin(tagRotation.getZ())
                                * modifier
                        + Constants.FIELD_X_MIDPOINT,
                tagTarget.getY() + Constants.PATHFINDING_POST_BUFFER * Math.sin(tagRotation.getZ())
                        - Constants.PATHFINDING_SHIFT_FACTOR * Math.cos(tagRotation.getZ())
                                * modifier
                        + Constants.FIELD_Y_MIDPOINT,
                tagTarget.getZ(),
                tagRotation);

        Pose2d finalTarget = finaltarget3d.toPose2d();

        System.out.println(finalTarget);
        drivetrain.setLabel(finalTarget, "final");

        ///// STRUCTURING

        Command exactAlignFirst = new ExactAlign(drivetrain, target); // A precise alignment that is done before we raise the elevator (since we need clearance)
        Command exactAlignFinal = new ExactAlign(drivetrain, finalTarget); // A precise alignment that is done after we raise the elevator and align the arm

        Command driveBackDynamic = new DriveDynamicX(drivetrain, 0.6, -0.5);
        Command driveDynamic = new DriveDynamicX(drivetrain, 0.297, 0.3);

        Command alignComponents = new InstantCommand(() -> core.moveToSetpoint(reefHeight)); // I documented this method if you need more info, but aligns ele/arm/wrist

        Command retractComponents = new InstantCommand(() -> core.performRetract()); // Retract sequences. Also documented. Selected automatically based on what setpoint was last used.

        Command resetNavPilot = new InstantCommand(() -> updateGUI(0)); // For my GUI, ignore but leave in the sequential pls

        runningCommand = new SequentialCommandGroup(prepathfindingCommand, pathfindingCommand, exactAlignFirst,
                alignComponents, exactAlignFinal, retractComponents, resetNavPilot);

        runningCommand.schedule();

        updateGUI(4);

    }
}
