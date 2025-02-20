// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.digital;

import java.util.ArrayDeque;
import java.util.Deque;
import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
import frc.robot.utils.Apriltags;
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

    // Pathfind sequence command queue for DEBUGGING TODO: REMOVE
    Deque<Command> commandQueue = new ArrayDeque<Command>();
    public void executeCommandQueue() {
        if (!commandQueue.isEmpty()) {commandQueue.poll().schedule();}
    }

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
            if (isReef && (loc < 0 || loc > 7)) {
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
    public void queueFind(int posCode, Alignment alignment) {
        this.tagId = translateToTagId(posCode);
        this.alignment = alignment;
        this.locationLoaded = true;
        if (heightLoaded) {
            newExecuteSequence();
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
            newExecuteSequence();
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

    public void newExecuteSequence() {
        Pose3d tagTarget = Apriltags.getWeldedPosition(tagId); // Get the tag's position from welded map

        if (tagTarget == null) {
            System.out.println("TARGET IS NULL");
            updateGUI(1000);
            return;
        }

        updateGUI(3);

        // PATHFIND

        Rotation3d tagRotation = tagTarget.getRotation().plus(new Rotation3d(0, 0, Math.PI));

        int modifier = alignment.getModif();

        Pose3d pathfindTarget3d = new Pose3d(
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

        Pose2d pathfindTarget = pathfindTarget3d.toPose2d();

        PathConstraints constraints = new PathConstraints(
                Constants.PATHFINDING_MAX_VELOCITY,
                Constants.PATHFINDING_MAX_ACCELERATION,
                Constants.PATHFINDING_MAX_ROTATIONAL_VELOCITY,
                Constants.PATHFINDING_MAX_ROTATIONAL_ACCELERATION);

        System.out.println(pathfindTarget);
        drivetrain.setLabel(pathfindTarget, "pathfind_target");

        Command pathfindCommand = AutoBuilder.pathfindToPose(
                pathfindTarget,
                constraints,
                0);

        ExactAlign exactAlignCommand = new ExactAlign(drivetrain);

        DriveDynamicX driveForward = new DriveDynamicX(drivetrain, Constants.MIN_CAMERA_DISTANCE, 0.1); // TODO: Tune speed of drive

        Command alignComponents = new InstantCommand(() -> core.moveToSetpoint(reefHeight));
        Command retractComponents = new InstantCommand(() -> core.performRetract());

        Command resetNavPilot = new InstantCommand(() -> updateGUI(0));

        SequentialCommandGroup pathfindSequence = new SequentialCommandGroup(pathfindCommand, exactAlignCommand, driveForward, alignComponents, retractComponents, resetNavPilot);
        // pathfindSequence.schedule();

        commandQueue.clear();
        commandQueue.add(pathfindCommand);
        commandQueue.add(exactAlignCommand);
        commandQueue.add(driveForward);
        commandQueue.add(alignComponents);
        commandQueue.add(retractComponents);
        commandQueue.add(resetNavPilot);
    }

    // // Once both pathfind and align are queued, execute the sequence
    // public void executeSequence() {
    //     Pose3d tagTarget = Apriltags.getWeldedPosition(tagId); // Get the tag's position from welded map

    //     if (tagTarget == null) {
    //         System.out.println("TARGET IS NULL");
    //         updateGUI(1000);
    //         return;
    //     }

    //     updateGUI(3);

    //     ///// PRE PATHFIND

    //     Rotation3d tagRotation = tagTarget.getRotation().plus(new Rotation3d(0, 0, Math.PI));

    //     int modifier = alignment.getModif();

    //     Pose3d pretarget3d = new Pose3d(
    //             tagTarget.getX() + Constants.PATHFINDING_PRE_BUFFER * Math.cos(tagRotation.getZ())
    //                     + Constants.PATHFINDING_SHIFT_FACTOR * Math.sin(tagRotation.getZ())
    //                             * modifier
    //                     + Constants.FIELD_X_MIDPOINT,
    //             tagTarget.getY() + Constants.PATHFINDING_PRE_BUFFER * Math.sin(tagRotation.getZ())
    //                     - Constants.PATHFINDING_SHIFT_FACTOR * Math.cos(tagRotation.getZ())
    //                             * modifier
    //                     + Constants.FIELD_Y_MIDPOINT,
    //             tagTarget.getZ(),
    //             tagRotation);

    //     Pose2d pretarget = pretarget3d.toPose2d();

    //     PathConstraints constraints = new PathConstraints(
    //             Constants.PATHFINDING_MAX_VELOCITY,
    //             Constants.PATHFINDING_MAX_ACCELERATION,
    //             Constants.PATHFINDING_MAX_ROTATIONAL_VELOCITY,
    //             Constants.PATHFINDING_MAX_ROTATIONAL_ACCELERATION);

    //     System.out.println(pretarget);
    //     // drivetrain.setLabel(pretarget, "pre");

    //     Command prepathfindingCommand = AutoBuilder.pathfindToPose(
    //             pretarget,
    //             constraints,
    //             0);

    //     ///// MAIN PATHFIND

    //     System.out.println(" --- TAG DATA --- ");
    //     System.out.println(tagTarget.getX());
    //     System.out.println(tagTarget.getY());
    //     System.out.println(tagTarget.getZ());
    //     System.out.println(tagTarget.getRotation().getX());
    //     System.out.println(tagTarget.getRotation().getY());
    //     System.out.println(tagTarget.getRotation().getZ());

    //     Pose3d target3d = new Pose3d(
    //             tagTarget.getX() + Constants.PATHFINDING_FRONT_BUFFER * Math.cos(tagRotation.getZ())
    //                     + Constants.PATHFINDING_SHIFT_FACTOR * Math.sin(tagRotation.getZ())
    //                             * modifier
    //                     + Constants.FIELD_X_MIDPOINT,
    //             tagTarget.getY() + Constants.PATHFINDING_FRONT_BUFFER * Math.sin(tagRotation.getZ())
    //                     - Constants.PATHFINDING_SHIFT_FACTOR * Math.cos(tagRotation.getZ())
    //                             * modifier
    //                     + Constants.FIELD_Y_MIDPOINT,
    //             tagTarget.getZ(),
    //             tagRotation);

    //     Pose2d target = target3d.toPose2d();

    //     System.out.println(target);
    //     // drivetrain.setLabel(target, "main");

    //     Command pathfindingCommand = AutoBuilder.pathfindToPose(
    //             target,
    //             constraints,
    //             0);

    //     ///// FINAL PATHFIND

    //     Pose3d finaltarget3d = new Pose3d(
    //             tagTarget.getX() + Constants.PATHFINDING_POST_BUFFER * Math.cos(tagRotation.getZ())
    //                     + Constants.PATHFINDING_SHIFT_FACTOR * Math.sin(tagRotation.getZ())
    //                             * modifier
    //                     + Constants.FIELD_X_MIDPOINT,
    //             tagTarget.getY() + Constants.PATHFINDING_POST_BUFFER * Math.sin(tagRotation.getZ())
    //                     - Constants.PATHFINDING_SHIFT_FACTOR * Math.cos(tagRotation.getZ())
    //                             * modifier
    //                     + Constants.FIELD_Y_MIDPOINT,
    //             tagTarget.getZ(),
    //             tagRotation);

    //     Pose2d finalTarget = finaltarget3d.toPose2d();

    //     System.out.println(finalTarget);
    //     drivetrain.setLabel(finalTarget, "final");

    //     ///// STRUCTURING

    //     Command exactAlignFirst = new ExactAlign(drivetrain, tagId, Constants.PATHFINDING_SHIFT_FACTOR * modifier, Constants.PATHFINDING_FRONT_BUFFER); // A precise alignment that is done before we raise the elevator (since we need clearance)
    //     Command exactAlignFinal = new ExactAlign(drivetrain, tagId, Constants.PATHFINDING_SHIFT_FACTOR * modifier, Constants.PATHFINDING_POST_BUFFER).withTimeout(5); // A precise alignment that is done after we raise the elevator and align the arm

    //     Command driveBackDynamic = new DriveDynamicX(drivetrain, 0.6, -0.5);
    //     Command driveDynamic = new DriveDynamicX(drivetrain, 0.297, 0.3);

    //     Command alignComponents = new InstantCommand(() -> core.moveToSetpoint(reefHeight)); // I documented this method if you need more info, but aligns ele/arm/wrist

    //     Command retractComponents = new InstantCommand(() -> core.performRetract()); // Retract sequences. Also documented. Selected automatically based on what setpoint was last used.

    //     Command resetNavPilot = new InstantCommand(() -> updateGUI(0)); // For my GUI, ignore but leave in the sequential pls

    //     // runningCommand = new SequentialCommandGroup(prepathfindingCommand, exactAlignFirst,
    //     //         alignComponents, exactAlignFinal, retractComponents, resetNavPilot);

    //     runningCommand = new SequentialCommandGroup(exactAlignFinal);

    //     runningCommand.schedule();

    //     updateGUI(4);
    // }
    
    public int translateToTagId(int posCode) {
        final Optional<DriverStation.Alliance> driverStationAlliance = DriverStation.getAlliance();
        if (!driverStationAlliance.isPresent()) { return -1; }
        final boolean isRed = driverStationAlliance.get() == Alliance.Red;
        
        int tagId = -1;
        switch (posCode) {
            case 1:
                tagId = isRed ? 10 : 18;
                break;
            case 2:
                tagId = isRed ? 11 : 17;
                break;
            case 3:
                tagId = isRed ? 6 : 22;
                break;
            case 4:
                tagId = isRed ? 7 : 21;
                break;
            case 5:
                tagId = isRed ? 8 : 20;
                break;
            case 6:
                tagId = isRed ? 9 : 19;
                break;
            case 10:
                tagId = isRed ? 1 : 12;
                break;
            case 11:
                tagId = isRed ? 2 : 13;
                break;
        }

        return tagId;
    }
}