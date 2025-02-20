// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.seafinder.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;

public class Pathfind extends Command {

    CommandSwerveDrivetrain drivetrain;

    Pose2d target;

    double maxVelocity = 3.0;
    double maxAcceleration = 4.0;
    double maxAngularVelocity = Units.degreesToRadians(540);
    double maxAngularAcceleration = Units.degreesToRadians(720);

    double endVelocity = 0.0;

    Command pathfindingCommand;

    /** Creates a new Pathfind. */
    public Pathfind(CommandSwerveDrivetrain drivetrain, Pose2d target) {

        System.out.println("init target: " + target);

        this.drivetrain = drivetrain;
        this.target = target;

        // Make sure no values are null
        if (drivetrain == null) {
            throw new IllegalArgumentException("Drivetrain cannot be null");
        } else if (target == null) {
            throw new IllegalArgumentException("Target cannot be null");
        }

        addRequirements(drivetrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

        // Create the constraints to use while pathfinding
        PathConstraints constraints = new PathConstraints(
                maxVelocity, maxAcceleration,
                maxAngularVelocity, maxAngularAcceleration);

        System.out.println(target);

        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        pathfindingCommand = AutoBuilder.pathfindToPose(
                target,
                constraints,
                endVelocity);

        pathfindingCommand.schedule();

        System.out.println("PATHFIND TO " + target.toString() + " STARTED");
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        System.out.println("PATHFIND TO " + target.toString() + " ENDED");
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return pathfindingCommand.isFinished();
    }
}
