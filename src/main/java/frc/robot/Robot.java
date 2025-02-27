// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.seafinder.PathfinderSubsystem.Alignment;
import frc.robot.seafinder.commands.InitRaiseArm;
import frc.robot.seafinder.commands.ZeroElevator;
import frc.robot.seafinder.utils.AStar;
import frc.robot.seafinder.utils.Apriltags;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private final Core m_core;

    public Robot() {
        m_core = new Core();
        Apriltags.loadField();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void disabledExit() {
    }

    @Override
    public void autonomousInit() {
        // Raising elevator a little
        InstantCommand raiseElevator = new InstantCommand(() -> m_core.getElevatorSubsystem().raise(4));
        raiseElevator.schedule();

        // Waiting until arm is raised
        WaitCommand waitCommand = new WaitCommand(0.3);
        waitCommand.schedule();

        // Creating pathfinder to get out of starting position
        m_core.getPathfinderSubsystem().clearSequence();
        int[][] path = m_core.getNavInterfaceSubsystem().loadPathData();
        System.out.println("Path loaded: " + path.length);
        InstantCommand pathfinder = new InstantCommand(() -> m_core.getPathfinderSubsystem().executePath(path));
        
        // Raising arm and zeroing elevator
        InitRaiseArm moveArm = new InitRaiseArm(m_core.getArmSubsystem(), Constants.ARM_HORIZONTAL);
        ZeroElevator zeroElevator = new ZeroElevator(m_core.getElevatorSubsystem());
        SequentialCommandGroup sequentialCommandGroup = new SequentialCommandGroup(moveArm, zeroElevator);
        
        // Running pathfinder and arm raise/elevator zero in parallel
        ParallelCommandGroup parallelCommandGroup = new ParallelCommandGroup(pathfinder, sequentialCommandGroup);
        parallelCommandGroup.schedule();

    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void autonomousExit() {
    }

    @Override
    public void teleopInit() {
        // Raising elevator a little
        Command raiseElevator = new InstantCommand(() -> m_core.getElevatorSubsystem().raise(4));

        // Waiting until elevator is raised
        Command waitCommand = new WaitCommand(0.3);

        // Raising arm and zeroing elevator
        Command moveArm = new InitRaiseArm(m_core.getArmSubsystem(), Constants.SETPOINT_MIN.getArm());
        Command zeroElevator = new ZeroElevator(m_core.getElevatorSubsystem());

        // Running arm raise and elevator zero
        SequentialCommandGroup commandGroup = new SequentialCommandGroup(raiseElevator, waitCommand, moveArm, zeroElevator);
        commandGroup.schedule();
    }

    @Override
    public void teleopPeriodic() {
        m_core.corePeriodic();
    }

    @Override
    public void teleopExit() {
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void testExit() {
    }

    @Override
    public void simulationPeriodic() {
    }
}
