
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
import frc.robot.Constants;
import frc.robot.Core;
import frc.robot.seafinder.commands.IntakeCommand;
import frc.robot.seafinder2.utils.Target.Side;
import frc.robot.seafinder2.utils.Apriltags;
import frc.robot.seafinder2.commands.ExactAlign;
import frc.robot.seafinder2.utils.Target;
import frc.robot.seafinder2.utils.Target.Height;
import frc.robot.seafinder2.utils.Target.Location;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;

public class PathfinderSubsystem {

    private Core core;
    private CommandSwerveDrivetrain drivetrain;

    private Target target;

    private Command runningCommand; // Keep track of the currently running command so we can override it later

    public PathfinderSubsystem(Core core) {
        this.core = core;
        this.drivetrain = core.getDrivetrain();

        target = new Target(core);
    }

    // Queue a pathfind; done by clicking a button on the panel
    public void queueFind(Location location) {
        target.setLocation(location);
        if (target.isComputed()) {
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

        Pose3d pathfindTarget3d = new Pose3d(
                tagTarget.getX() + Constants.PATHFINDING_PRE_BUFFER * Math.cos(tagRotation.getZ())
                        + Math.sin(tagRotation.getZ()) + Constants.FIELD_X_MIDPOINT,
                tagTarget.getY() + Constants.PATHFINDING_PRE_BUFFER * Math.sin(tagRotation.getZ())
                        - Math.cos(tagRotation.getZ()) + Constants.FIELD_Y_MIDPOINT,
                tagTarget.getZ(),
                tagRotation);

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

        // ALIGN - Both
        Command alignComponents = new InstantCommand(() -> {
            core.moveToSetpoint(target.getSetpoint());
        });

        Command retractComponents = target.getRetractCommand();

        if (target.isReef()) { // Reef

            Command exactAlign = new ExactAlign(drivetrain, target.getTagRelativePose());

            runningCommand = new SequentialCommandGroup(
                    lowerRobot,
                    pathfindCommand,
                    alignComponents,
                    exactAlign,
                    retractComponents);

            runningCommand.schedule();

        } else { // Human Station

            Command intakeCommand = new IntakeCommand(core.getManipulatorSubsystem());

            runningCommand = new SequentialCommandGroup(
                    lowerRobot,
                    pathfindCommand,
                    alignComponents,
                    // fieldAlign,
                    intakeCommand,
                    retractComponents
            );

            runningCommand.schedule();

        }
    }

}