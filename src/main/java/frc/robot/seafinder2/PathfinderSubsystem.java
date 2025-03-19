
package frc.robot.seafinder2;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.DriverStation;
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
import frc.robot.seafinder2.commands.FieldAlign;
import frc.robot.seafinder2.commands.StaticBack;
import frc.robot.seafinder2.commands.StopDrivetrain;
import frc.robot.seafinder2.commands.limbControl.ManipulatorCommand;
import frc.robot.seafinder2.commands.limbControl.IntakeOuttakeCommand;
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

    private boolean skipAStar = false;

    public Command intakeCommand;
    private Command runningCommand; // Keep track of the currently running command so we can override it later

    public SequentialCommandGroup autoSequence;
   // public SequentialCommandGroup autoSequence2;

    public PathfinderSubsystem(Core core) {
        this.core = core;
        this.drivetrain = core.getDrivetrain();

        autoSequence = new SequentialCommandGroup();
      //  autoSequence2 = new SequentialCommandGroup();

        target = new Target(core);
    }

    // Queue a pathfind; done by clicking a button on the panel
    public void queueFind(Location location) {
        skipAStar = false;
        System.out.println("queueFind ran");
        target.setLocation(location);
        if (target.isComputed()) {
            System.out.println("Target was computed");
            executeSequence(target);
            target = new Target(core);
        }
    }

    public void queueFind(Location location, boolean skipAStar) {
        this.skipAStar = skipAStar;
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

       //double hpExtraPadding = target.isReef() ? 0 : -1.5;  This seems to leave it too far back, reducing
        double hpExtraPadding = target.isReef() ? 0 : -.5;

        // Pose3d pathfindTarget3d = new Pose3d(
        //         tagTarget.getX() + SF2Constants.SEAFINDER2_ASTAR_PADDING * Math.cos(tagRotation.getZ())
        //                 + Math.sin(tagRotation.getZ()) + Constants.FIELD_X_MIDPOINT,
        //         tagTarget.getY() + SF2Constants.SEAFINDER2_ASTAR_PADDING * Math.sin(tagRotation.getZ())
        //                 - Math.cos(tagRotation.getZ()) + Constants.FIELD_Y_MIDPOINT,
        //         tagTarget.getZ(),
        //         tagRotation);

        double fieldX = tagTarget.getX() + 
        (SF2Constants.SEAFINDER2_ASTAR_PADDING + hpExtraPadding) * Math.cos(tagRotation.getZ()) + 
        Constants.FIELD_X_MIDPOINT;

        double fieldY = tagTarget.getY() +
        (SF2Constants.SEAFINDER2_ASTAR_PADDING + hpExtraPadding) * Math.sin(tagRotation.getZ()) + 
        Constants.FIELD_Y_MIDPOINT;


        Pose3d pathfindTarget3d = new Pose3d(
            fieldX,
            fieldY,
            tagTarget.getZ(),
            tagRotation
        );

        Pose2d pathfindTarget = pathfindTarget3d.toPose2d();

        PathConstraints constraints = new PathConstraints(SF2Constants.SEAFINDER2_MAX_VELOCITY,
                SF2Constants.SEAFINDER2_MAX_ACCELERATION, SF2Constants.SEAFINDER2_MAX_ROTATIONAL_VELOCITY,
                SF2Constants.SEAFINDER2_MAX_ROTATIONAL_ACCELERATION);

        System.out.println(pathfindTarget);
        drivetrain.setLabel(pathfindTarget, "pathfind_target");

        Command pathfindMovementCommand = AutoBuilder.pathfindToPose(
                pathfindTarget,
                constraints,
                0);
        Command stopDrivetrainCommand = new StopDrivetrain(drivetrain);
        Command pathfindCommand = new SequentialCommandGroup(pathfindMovementCommand, stopDrivetrainCommand);

        if (target.isReef()) {
            System.out.println("RUNNING REEF SEQUENCE");
            drivetrain.setLabel(target.getTagRelativePose().getPose2d(), "EXA");

            Command alignComponents = new ParallelCommandGroup(
                new ElevatorCommand(core.getElevatorSubsystem(), target.getSetpoint().getElevator(), true),
                new ArmCommand(core.getArmSubsystem(),target.getSetpoint().getArm(), true),
                new WristCommand(core.getArmSubsystem(), target.getSetpoint().getWrist(), true)
            );
            Command exactAlign = new SequentialCommandGroup(
                new WaitCommand(0.0), 
                new ExactAlign(drivetrain, target.getTagRelativePose())
            );
            Command exactAlignAndAlignComponents = new ParallelCommandGroup(exactAlign, alignComponents);

            Command retractComponents = target.getRetractCommand();

            SequentialCommandGroup commandGroup = new SequentialCommandGroup();
            if (skipAStar) {
                commandGroup.addCommands(
                    exactAlignAndAlignComponents,
                    retractComponents
                );
            } else {
                commandGroup.addCommands(
                    pathfindCommand,
                    exactAlignAndAlignComponents,
                    retractComponents
                );
            }

            if (DriverStation.isAutonomous()) {
                autoSequence = commandGroup;
            } else {
                commandGroup.schedule();
            }
        } else {
            System.out.println("RUNNING HUMAN STATION SEQUENCE");

            Command alignComponents = new ParallelCommandGroup(
                new ElevatorCommand(core.getElevatorSubsystem(), target.getSetpoint().getElevator(), true),
                new ManipulatorCommand(core.getArmSubsystem(), target.getSetpoint().getArm(), true, target.getSetpoint().getWrist(), true)
            );
            Command fieldAlign = new FieldAlign(drivetrain, target.getTag(), fieldX, fieldY, tagRotation.getZ());

            // Command intakeCommand = new IntakeCommand(core.getManipulatorSubsystem());
            Command pathFindAndAlignComponents = new ParallelCommandGroup(
                // pathfindCommand.until(() -> drivetrain.robotNearHP()),
                pathfindCommand.until(() -> drivetrain.robotNearHP()),
                alignComponents
            );

            Command canRangeForward = new CanRangeDynamicForward(drivetrain);
            intakeCommand = new IntakeCommand(core.getManipulatorSubsystem());

            Command staticBack = new StaticBack(drivetrain).withTimeout(0.5);
            Command wristToScoringPos = new InstantCommand(() -> core.getArmSubsystem().armGoTo(SF2Constants.WRIST_MIN_POSITION)); 

            SequentialCommandGroup commandGroup = new SequentialCommandGroup();
            commandGroup.addCommands(
                pathFindAndAlignComponents,
                fieldAlign,
                canRangeForward,
                intakeCommand,
                staticBack,
                wristToScoringPos
            );

            if (DriverStation.isAutonomous()) {
                autoSequence = commandGroup;
            } else {
                runningCommand.schedule();
            }
        }
    }
}