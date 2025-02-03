// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PathfindBasic extends InstantCommand {

  CommandSwerveDrivetrain drivetrain;

  Pose2d target;

  double maxVelocity = 3.0;
  double maxAcceleration = 4.0;
  double maxAngularVelocity = Units.degreesToRadians(540);
  double maxAngularAcceleration = Units.degreesToRadians(720);

  double endVelocity = 0.0;

  Command pathfindingCommand;

  public PathfindBasic(CommandSwerveDrivetrain drivetrain, Pose2d target) {

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
        maxAngularVelocity, maxAngularAcceleration
    );

    System.out.println(target);

    // Since AutoBuilder is configured, we can use it to build pathfinding commands
    pathfindingCommand = AutoBuilder.pathfindToPose(
        target,
        constraints,
        endVelocity
    );

    pathfindingCommand.schedule();

    System.out.println("PATHFIND TO " + target.toString() + " STARTED");
  }
}
