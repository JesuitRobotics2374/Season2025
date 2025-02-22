// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.seafinder;

import java.util.ArrayDeque;
import java.util.Deque;
import java.util.Optional;

import com.ctre.phoenix6.wpiutils.AutoFeedEnable;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Core;
import frc.robot.seafinder.commands.CanRangeDynamicForward;
import frc.robot.seafinder.commands.DriveDynamicX;
import frc.robot.seafinder.commands.ExactAlignRot;
import frc.robot.seafinder.commands.ExactAlignXY;
import frc.robot.seafinder.commands.StaticBackCommand;
import frc.robot.seafinder.utils.Apriltags;
import frc.robot.seafinder.utils.Setpoint;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;

public class PathfinderSubsystem {

    private Core core; // Keep an instance of core for moveToSetpoint and performRetract
    private CommandSwerveDrivetrain drivetrain;

    private boolean locationLoaded = false; // Have we inputted a location?
    private int tagId; // The tag ID to navigate to
    private Alignment alignment; // The alignment of the robot to the tag (left/right)

    private boolean heightLoaded = false; // Have we inputted a height?
    private Setpoint reefHeight; // What setpoint should we move to? (T1/2/3/4)

    private Command runningCommand; // Keep track of the currently running command so we can override it later

    private boolean isTeleop = false;
    private SequentialCommandGroup autoCommandSequence;

    // Pathfind sequence command queue for DEBUGGING TODO: REMOVE
    private Deque<Command> commandQueue = new ArrayDeque<Command>();
    private Command prevComand = null;

    public void executeCommandQueue() {
        if (prevComand != null && !prevComand.isFinished() && !commandQueue.isEmpty()) {
            return;
        }

        if (!commandQueue.isEmpty()) {
            prevComand = commandQueue.poll();
            prevComand.schedule();
        }
    }

    // Alignment data structure
    public enum Alignment {
        LEFT, RIGHT, CENTER;

        private double modif;

        static {
            LEFT.modif = Constants.PATHFINDING_LEFT_SHIFT_FACTOR;
            RIGHT.modif = Constants.PATHFINDING_RIGHT_SHIFT_FACTOR;
            CENTER.modif = (Constants.PATHFINDING_LEFT_SHIFT_FACTOR + Constants.PATHFINDING_RIGHT_SHIFT_FACTOR) / 2;
        }

        public double getOffset() {
            return modif;
        }

        public static Alignment parseTopology(boolean isReef, int loc) {
            if (isReef && (loc < 0 || loc > 7)) {
                throw new IllegalArgumentException("Invalid topology");
            }
            return isReef ? ((loc % 2 == 0) ? LEFT : RIGHT) : CENTER;
        }
    }

    // Subsystem is now constructed just once, at the top of Core with other
    // subsystems
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

        updateGUI(4);

        // PATHFIND

        Rotation3d tagRotation = tagTarget.getRotation().plus(new Rotation3d(0, 0, Math.PI));

        double offset = alignment.getOffset();

        Pose3d pathfindTarget3d = new Pose3d(
                tagTarget.getX() + Constants.PATHFINDING_PRE_BUFFER * Math.cos(tagRotation.getZ())
                        + Math.sin(tagRotation.getZ()) * offset
                        + Constants.FIELD_X_MIDPOINT,
                tagTarget.getY() + Constants.PATHFINDING_PRE_BUFFER * Math.sin(tagRotation.getZ())
                        - Math.cos(tagRotation.getZ()) * offset
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
        // drivetrain.setLabel(pathfindTarget, "pathfind_target");

        Command pathfindCommand = AutoBuilder.pathfindToPose(
                pathfindTarget,
                constraints,
                0);

        ExactAlignRot exactAlignCommandRot = new ExactAlignRot(drivetrain, tagId, offset);
        ExactAlignXY exactAlignCommandXY = new ExactAlignXY(drivetrain, tagId, offset);

        CanRangeDynamicForward dynamicForwardCommand = new CanRangeDynamicForward(drivetrain);

        // DriveDynamicX driveForward = new DriveDynamicX(drivetrain,
        // Constants.MIN_CAMERA_DISTANCE, 0.1); // TODO: Tune speed of drive

        Command alignComponents = new InstantCommand(() -> core.moveToSetpoint(reefHeight));
        Command retractComponents = new InstantCommand(() -> core.performRetract());

        Command pilotStateAlign = new InstantCommand(() -> updateGUI(5));
        Command pilotStateRot = new InstantCommand(() -> updateGUI(6));
        Command pilotStateXY = new InstantCommand(() -> updateGUI(7));
        Command pilotStateDynamic = new InstantCommand(() -> updateGUI(8));
        Command pilotStateRetract = new InstantCommand(() -> updateGUI(9));

        Command resetNavPilot = new InstantCommand(() -> updateGUI(0));

        // SequentialCommandGroup pathfindSequence = new
        // SequentialCommandGroup(pathfindCommand, exactAlignCommand, driveForward,
        // alignComponents, retractComponents, resetNavPilot);

        // if (isTeleop) {
        // pathfindSequence.schedule();
        // } else {
        // addToAutoSequence(exactAlignCommand);
        // }

        commandQueue.clear();
        prevComand = null;
        // commandQueue.add(pathfindCommand);
        // commandQueue.add(exactAlignCommand);
        // commandQueue.add(driveForward);
        // commandQueue.add(alignComponents);
        // commandQueue.add(retractComponents);
        // commandQueue.add(resetNavPilot);

        // SequentialCommandGroup testCommandGroup = new
        // SequentialCommandGroup(exactAlignCommand, dynamicForwardCommand,
        // resetNavPilot);
        SequentialCommandGroup testCommandGroup = new SequentialCommandGroup(pathfindCommand, pilotStateAlign,
                alignComponents, pilotStateRot, exactAlignCommandRot, pilotStateXY, exactAlignCommandXY,
                pilotStateDynamic, dynamicForwardCommand, pilotStateRetract, retractComponents, resetNavPilot);
        testCommandGroup.schedule();
    }

    public int translateToTagId(int posCode) {
        final Optional<DriverStation.Alliance> driverStationAlliance = DriverStation.getAlliance();
        if (!driverStationAlliance.isPresent()) {
            return -1;
        }
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

    public void teleopEnabled() {
        isTeleop = true;
        autoCommandSequence = null;
    }

    public void AutoEnabled() {
        autoCommandSequence.schedule();
    }

    public void AutoDisabled() {
        autoCommandSequence.cancel();
    }

    public void addToAutoSequence(Command command) {
        if (autoCommandSequence == null) {
            autoCommandSequence = new SequentialCommandGroup(command);
        } else {
            autoCommandSequence.andThen(command);
        }
    }
}