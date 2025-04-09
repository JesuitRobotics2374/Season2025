
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
import frc.robot.seafinder2.utils.Apriltags;
import frc.robot.seafinder2.commands.CanRangeDynamicForward;
import frc.robot.seafinder2.commands.ExactAlign;
import frc.robot.seafinder2.commands.FieldAlign;
import frc.robot.seafinder2.commands.StaticBack;
import frc.robot.seafinder2.commands.StopDrivetrain;
import frc.robot.seafinder2.commands.limbControl.ManipulatorCommand;
import frc.robot.seafinder2.commands.limbControl.NewOuttake;
import frc.robot.seafinder2.commands.limbControl.IntakeCommand;
import frc.robot.seafinder2.commands.limbControl.ArmCommand;
import frc.robot.seafinder2.commands.limbControl.ElevatorCommand;
import frc.robot.seafinder2.commands.limbControl.WristCommand;
import frc.robot.seafinder2.utils.Target;
import frc.robot.seafinder2.utils.Target.Height;
import frc.robot.seafinder2.utils.Target.Location;
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
        double padding = target.isReef() ? SF2Constants.SEAFINDER2_ASTAR_PADDING : SF2Constants.SEAFINDER2_ASTAR_PADDING_HP;

        // Pose3d pathfindTarget3d = new Pose3d(
        //         tagTarget.getX() + SF2Constants.SEAFINDER2_ASTAR_PADDING * Math.cos(tagRotation.getZ())
        //                 + Math.sin(tagRotation.getZ()) + Constants.FIELD_X_MIDPOINT,
        //         tagTarget.getY() + SF2Constants.SEAFINDER2_ASTAR_PADDING * Math.sin(tagRotation.getZ())
        //                 - Math.cos(tagRotation.getZ()) + Constants.FIELD_Y_MIDPOINT,
        //         tagTarget.getZ(),
        //         tagRotation);

        double fieldX = tagTarget.getX() + padding * Math.cos(tagRotation.getZ()) + Constants.FIELD_X_MIDPOINT;

        double fieldY = tagTarget.getY() + padding * Math.sin(tagRotation.getZ()) + Constants.FIELD_Y_MIDPOINT;


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

        Command pathfindCommand = AutoBuilder.pathfindToPose(
                pathfindTarget,
                constraints,
                0);
        Command stopDrivetrainCommand = new StopDrivetrain(drivetrain);

        Command alignComponents = new ParallelCommandGroup(
            new ElevatorCommand(core.getElevatorSubsystem(), target.getSetpoint().getElevator()  + 8, true),
            new SequentialCommandGroup(new WaitCommand(0.3), new ArmCommand(core.getArmSubsystem(),target.getSetpoint().getArm(), true)),
            new WristCommand(core.getArmSubsystem(), target.getSetpoint().getWrist(), true)
        );
        Command alignComponentsHP = new ParallelCommandGroup(
            new ElevatorCommand(core.getElevatorSubsystem(), target.getSetpoint().getElevator(), true),
            new ManipulatorCommand(core.getArmSubsystem(), target.getSetpoint().getArm(), true, target.getSetpoint().getWrist(), true)
        );

        Command retractComponents = target.getRetractCommand();

        if (target.isReef()) {
            System.out.println("RUNNING REEF SEQUENCE");
            
            // Command waitCommand = new WaitCommand(0.3);
            
            Command troughOuttake;
            if (target.getHeight().equals(Height.TROUGH)) {
                troughOuttake = new NewOuttake(core.getManipulatorSubsystem(), 1).withTimeout(0.3);
            } else {
                troughOuttake = new WaitCommand(0.3); // Otherwise use it as our wait
            }

            drivetrain.setLabel(target.getTagRelativePose().getPose2d(), "EXA");

            if (DriverStation.isAutonomous()) {
                Command exactAlign = new SequentialCommandGroup(new WaitCommand(0.0), new ExactAlign(drivetrain, target.getTagRelativePose()));
                Command alignBoth = new ParallelCommandGroup(exactAlign, alignComponents);

                if (skipAStar) {
                    autoSequence.addCommands(
                        // lowerRobot,
                        alignBoth,
                        troughOuttake, // Wait for elevator to stop moving/shaking
                        retractComponents
                    );
                } else {
                    autoSequence.addCommands(
                        // lowerRobot,
                        pathfindCommand,
                        stopDrivetrainCommand,
                        alignBoth,
                        troughOuttake, // Wait for elevator to stop moving/shaking
                        retractComponents
                );
                }
                // autoSequence.schedule();

            } else {
                Command exactAlign = new SequentialCommandGroup(new WaitCommand(0.5), new ExactAlign(drivetrain, target.getTagRelativePose()));
                Command alignBoth = new ParallelCommandGroup(exactAlign, alignComponents);

                if (skipAStar) {
                    runningCommand = new SequentialCommandGroup(
                        // lowerRobot,
                        alignBoth,
                        troughOuttake, // Wait for elevator to stop moving/shaking
                        retractComponents
                );
                } else {
                    runningCommand = new SequentialCommandGroup(
                        // lowerRobot,
                        pathfindCommand,                
                        stopDrivetrainCommand,
                        alignBoth,
                        troughOuttake, // Wait for elevator to stop moving/shaking
                        retractComponents
                );
                }
                runningCommand.schedule();

            }

            

        } else { // Human Station
            System.out.println("RUNNING HUMAN STATION SEQUENCE");

            Command hpFieldAlign = new FieldAlign(drivetrain, target.getTag(), fieldX, fieldY, tagRotation.getZ());

            // Command intakeCommand = new IntakeCommand(core.getManipulatorSubsystem());
            Command bothHP = new ParallelCommandGroup(
                // pathfindCommand.until(() -> drivetrain.robotNearHP()),
                new SequentialCommandGroup(new InstantCommand(() -> {System.out.println("ABDD START");}), pathfindCommand, (new InstantCommand(() -> {System.out.println("ABDD START");}))),
                alignComponentsHP
            );

            Command canForward = new CanRangeDynamicForward(drivetrain);
            intakeCommand = new IntakeCommand(core.getManipulatorSubsystem());

            Command staticBack = new StaticBack(drivetrain).withTimeout(0.2);

            if (DriverStation.isAutonomous()) {
                System.out.println("Auto to Human Station");
                autoSequence.addCommands(
                    bothHP,
                    //hpFieldAlign.until(() -> drivetrain.robotNearHP()),
                    hpFieldAlign,
                    stopDrivetrainCommand,
                    // alignComponentsHP,
                    canForward,
                    intakeCommand,
                    staticBack
                    // wristToScoringPosCommand 
                    // retractComponents
            );
            // autoSequence.schedule();
            } else {
                runningCommand = new SequentialCommandGroup(
                    bothHP,
                    hpFieldAlign.until(() -> drivetrain.robotNearHP()),
                    stopDrivetrainCommand,
                    // fieldAlign,
                    // alignComponentsHP,
                    canForward,
                    intakeCommand,
                    staticBack
                    // wristToScoringPosCommand 
                    // retractComponents
            );
            runningCommand.schedule();
            }
        }
    }

}