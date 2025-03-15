
package frc.robot.seafinder2;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Core;
import frc.robot.seafinder2.utils.Target.Side;
import frc.robot.seafinder2.utils.Apriltags;
import frc.robot.seafinder2.commands.CanRangeDynamicForward;
import frc.robot.seafinder2.commands.ExactAlign;
import frc.robot.seafinder2.commands.StaticBack;
import frc.robot.seafinder2.commands.StopDrivetrain;
import frc.robot.seafinder2.commands.limbControl.ManipulatorCommand;
import frc.robot.seafinder2.commands.limbControl.IntakeCommand;
import frc.robot.seafinder2.commands.limbControl.ArmCommand;
import frc.robot.seafinder2.commands.limbControl.ElevatorCommand;
import frc.robot.seafinder2.commands.limbControl.WristCommand;
import frc.robot.seafinder2.utils.Target;
import frc.robot.seafinder2.utils.Target.Height;
import frc.robot.seafinder2.utils.Target.Location;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;

public class PathfinderSubsystem {

    private Core core;
    private CommandSwerveDrivetrain drivetrain;

    private Target target;

    public Command intakeCommand;
    private Command runningCommand; // Keep track of the currently running command so we can override it later

    public PathfinderSubsystem(Core core) {
        this.core = core;
        this.drivetrain = core.getDrivetrain();

        target = new Target(core);
    }

    // Queue a pathfind; done by clicking a button on the panel
    public void queueFind(Location location) {
        System.out.println("queueFind ran");
        target.setLocation(location);
        if (target.isComputed()) {
            System.out.println("Target was computed");
            executeSequence(target);
            target = new Target(core);
        }
    }

    // Queue a reef location; done by clicking a button on the panel
    public void queueAlign(Height height) {
        target.setHeight(height);
        if (target.isComputed()) {
            executeSequence(target);
            target = new Target(core);
        }
    }

    // Unused for now, but should be implimented
    public void stopAll() {
        if (runningCommand != null) {
            runningCommand.cancel();
            runningCommand = null;
        }
    }

    public void executeSequence(Target target) {

        System.out.println(target.getTag());

        // Get the tag's position from welded map
        Pose3d tagTarget = Apriltags.getWeldedPosition(target.getTag());
        if (tagTarget == null) {
            System.out.println("TARGET IS NULL");
            return;
        }

        // LOWER - Both
        Command lowerRobot = new InstantCommand(() -> core.moveToSetpoint(SF2Constants.SETPOINT_MIN));

        // PATHFIND - Both
        Rotation3d tagRotation = tagTarget.getRotation().plus(new Rotation3d(0, 0, Math.PI));

        // Pose3d pathfindTarget3d = new Pose3d(
        //         tagTarget.getX() + SF2Constants.SEAFINDER2_ASTAR_PADDING * Math.cos(tagRotation.getZ())
        //                 + Math.sin(tagRotation.getZ()) + Constants.FIELD_X_MIDPOINT,
        //         tagTarget.getY() + SF2Constants.SEAFINDER2_ASTAR_PADDING * Math.sin(tagRotation.getZ())
        //                 - Math.cos(tagRotation.getZ()) + Constants.FIELD_Y_MIDPOINT,
        //         tagTarget.getZ(),
        //         tagRotation);

        Pose3d pathfindTarget3d = new Pose3d(
            tagTarget.getX() + 
                SF2Constants.SEAFINDER2_ASTAR_PADDING * Math.cos(tagRotation.getZ()) + 
                Constants.FIELD_X_MIDPOINT,
            tagTarget.getY() +
                SF2Constants.SEAFINDER2_ASTAR_PADDING * Math.sin(tagRotation.getZ()) + 
                Constants.FIELD_Y_MIDPOINT,
            tagTarget.getZ(),
            tagRotation
        );

        Pose2d pathfindTarget = pathfindTarget3d.toPose2d();

        PathConstraints constraints = new PathConstraints(SF2Constants.SEAFINDER2_MAX_VELOCITY,
                SF2Constants.SEAFINDER2_MAX_ACCELERATION, SF2Constants.SEAFINDER2_MAX_ROTATIONAL_VELOCITY,
                SF2Constants.SEAFINDER2_MAX_ROTATIONAL_ACCELERATION);

        System.out.println(pathfindTarget);
        drivetrain.setLabel(pathfindTarget, "pathfind_target");

        Command pathfindCommand = AutoBuilder.pathfindToPose(
                pathfindTarget,
                constraints,
                0);
        Command stopDrivetrainCommand = new StopDrivetrain(drivetrain);

        // ALIGN - Both
        // Command alignComponents = new InstantCommand(() -> {
        //     core.moveToSetpoint(target.getSetpoint());
        // });

        // elevatorSubsystem.elevatorGoToDouble(setpoint.getElevator());
        // armSubsystem.armGoTo(setpoint.getArm());
        // armSubsystem.wristGoTo(setpoint.getWrist());

        Command alignComponents = new ParallelCommandGroup(
            new ElevatorCommand(core.getElevatorSubsystem(), target.getSetpoint().getElevator(), true),
            new ArmCommand(core.getArmSubsystem(),target.getSetpoint().getArm(), true),
            new WristCommand(core.getArmSubsystem(), target.getSetpoint().getWrist(), true)
        );
        Command alignComponentsHP = new ParallelCommandGroup(
            new ElevatorCommand(core.getElevatorSubsystem(), target.getSetpoint().getElevator(), true),
            new ManipulatorCommand(core.getArmSubsystem(), target.getSetpoint().getArm(), true, target.getSetpoint().getWrist(), true)
        );

        Command retractComponents = target.getRetractCommand();
        Command wristToScoringPosCommand = new WristCommand(core.getArmSubsystem(), SF2Constants.WRIST_MIN_POSITION, true); 

        if (target.isReef()) {
            System.out.println("RUNNING REEF SEQUENCE");
            
            Command exactAlign = new SequentialCommandGroup(new WaitCommand(1), new ExactAlign(drivetrain, target.getTagRelativePose()));
            Command alignBoth = new ParallelCommandGroup(exactAlign, alignComponents);
            Command waitCommand = new WaitCommand(0.3);

            drivetrain.setLabel(target.getTagRelativePose().getPose2d(), "EXA");

            runningCommand = new SequentialCommandGroup(
                    // lowerRobot,
                    pathfindCommand,
                    stopDrivetrainCommand,
                    alignBoth,
                    waitCommand, // Wait for elevator to stop moving/shaking
                    retractComponents
            );

            runningCommand.schedule();

        } else { // Human Station
            System.out.println("RUNNING HUMAN STATION SEQUENCE");

            // Command intakeCommand = new IntakeCommand(core.getManipulatorSubsystem());
            Command bothHP = new ParallelCommandGroup(
                pathfindCommand.until(() -> drivetrain.robotNearHP()),
                alignComponentsHP
            );

            Command canForward = new CanRangeDynamicForward(drivetrain);
            intakeCommand = new IntakeCommand(core.getManipulatorSubsystem());

            Command staticBack = new StaticBack(drivetrain).withTimeout(0.5);
            runningCommand = new SequentialCommandGroup(
                    // lowerRobot,
                    // pathfindCommand,
                    bothHP,
                    stopDrivetrainCommand,
                    // fieldAlign,
                    // alignComponentsHP,
                    canForward,
                    intakeCommand,
                    staticBack,
                    wristToScoringPosCommand 
                    // retractComponents
            );

            

            runningCommand.schedule();

        }
    }
}