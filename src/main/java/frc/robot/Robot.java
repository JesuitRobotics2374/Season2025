// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import org.photonvision.EstimatedRobotPose;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.commands.PathfindingCommand;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.seafinder2.utils.Apriltags;
import frc.robot.seafinder2.utils.Target.Height;
import frc.robot.seafinder2.utils.Target.Landmark;
import frc.robot.seafinder2.utils.Target.Location;
import frc.robot.seafinder2.utils.Target.Side;
import frc.robot.subsystems.VisionSubsystem;

public class Robot extends TimedRobot {

    private final Core m_core;

    public Robot() {
        m_core = new Core();
        Apriltags.loadField();

        m_core.getDrivetrain().seedRobotAuto();

        PathfindingCommand.warmupCommand().schedule();

        VisionSubsystem.initializeVisionSubsystem();
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
        m_core.getDrivetrain().configNeutralMode(NeutralModeValue.Brake);
        CommandScheduler.getInstance().schedule(m_core.getElevatorSubsystem().haltAll());

        System.out.println("Auto-Iit");
        m_core.getDrivetrain().seedRobotAuto();

        List<EstimatedRobotPose> estimatedRobotPoses = VisionSubsystem.getGlobalFieldPoses();

        for (EstimatedRobotPose estimatedRobotPose : estimatedRobotPoses) {
            if (estimatedRobotPose != null) {
                // field.getObject("Vision" + displayCounter).setPose(fp.pose);
                m_core.getDrivetrain().alignToVision(estimatedRobotPose, false);
            }
        }

        CommandScheduler.getInstance().schedule(m_core.getElevatorSubsystem().incrementUp(1)); //This number may need to change

        // First Auto
        m_core.pathfinderSubsystem.queueFind(new Location(Landmark.REEF_BACK_RIGHT, Side.LEFT), true);
        m_core.pathfinderSubsystem.queueAlign(Height.BRANCH_L4);

        // Human Station
        m_core.pathfinderSubsystem.queueFind(new Location(Landmark.STATION_RIGHT));

        // Second Auto L1
        // m_core.pathfinderSubsystem.queueFind(new Location(Landmark.REEF_FRONT_RIGHT,
        // Side.LEFT), false);
        // m_core.pathfinderSubsystem.queueAlign(Height.BRANCH_L4);

        // Second Auto L4
        m_core.pathfinderSubsystem.queueFind(new Location(Landmark.REEF_FRONT_RIGHT, Side.LEFT), false);
        m_core.pathfinderSubsystem.queueAlign(Height.BRANCH_L4);

        // all command should be in pathfinder auto sequence now
        m_core.autoCommandGroup = new SequentialCommandGroup(m_core.getPathfinderSubsystem().autoSequence);
        m_core.autoCommandGroup.schedule();

        System.out.println("Auto schedule complete");

        // Command waitcmd = new WaitCommand(5);

        // Command scheduleAutoInit2 = new InstantCommand(() ->
        // m_core.getPathfinderSubsystem().autoSequence.schedule());

        // m_core.autoCommandGroup = new SequentialCommandGroup(seedForAuto,
        // snapToLimelight, raiseElevator, autoLocation, autoHeight, scheduleAutoInit,
        // hp, scheduleAutoInit2);

        // while (!m_core.autoCommandGroup.isFinished());

        // InitRaiseArm moveArm = new InitRaiseArm(m_core.getArmSubsystem());
        // ZeroElevator zeroElevator = new ZeroElevator(m_core.getElevatorSubsystem());

        // SequentialCommandGroup commandGroup = new SequentialCommandGroup(moveArm,
        // zeroElevator, pathfinder)

        // m_core.getPathfinderSubsystem().clearSequence();
        // int[][] path = m_core.getNavInterfaceSubsystem().loadPathData();
        // System.out.println("Path loaded: " + path.length);
        // InstantCommand pathfinder = new InstantCommand(() ->
        // m_core.getPathfinderSubsystem().executePath(path));

        // Command moveForward = new TimedForward(m_core.getDrivetrain(), 1.5);
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void autonomousExit() {
        m_core.getDrivetrain().configNeutralMode(NeutralModeValue.Brake);
        CommandScheduler.getInstance().schedule(m_core.getElevatorSubsystem().haltAll());
    }

    @Override
    public void teleopInit() {
        m_core.getDrivetrain().configNeutralMode(NeutralModeValue.Coast); // TODO: REMOVE
        // System.out.println("Teleop-Iit");
        // InstantCommand raiseElevator = new InstantCommand(() ->
        // m_core.getElevatorSubsystem().raise(5));
        // WaitCommand waitCommand = new WaitCommand(0.7);
        // InitRaiseArm moveArm = new InitRaiseArm(m_core.getArmSubsystem());
        // // InstantCommand raiseArm = new InstantCommand( () ->
        // m_core.getArmSubsystem().armGoTo(18.68));
        // InstantCommand lowerToLimit = new InstantCommand( () ->
        // m_core.getElevatorSubsystem().lowerToLimit() );
        // SequentialCommandGroup commandGroup = new
        // SequentialCommandGroup(raiseElevator, waitCommand, moveArm, lowerToLimit);
        // commandGroup.schedule();
    }

    @Override
    public void teleopPeriodic() {
        m_core.corePeriodic();
    }

    @Override
    public void teleopExit() {
        m_core.getDrivetrain().configNeutralMode(NeutralModeValue.Brake);
        CommandScheduler.getInstance().schedule(m_core.getElevatorSubsystem().haltAll());
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
