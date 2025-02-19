// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.digital.PathfinderSubsystem.Alignment;
import frc.robot.utils.AStar;
import frc.robot.utils.Apriltags;

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
        // m_autonomousCommand = m_core.getAutonomousCommand();

        // if (m_autonomousCommand != null) {
        // m_autonomousCommand.schedule();
        // }

        // m_core.instantFind(18, Alignment.LEFT);

        // m_robotContainer.getPath("Prec-BLUE-TOP").schedule();

        // m_robotContainer.doPathfindToPath("Prec-BLUE-TOP");

    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void autonomousExit() {
    }

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
    }

    @Override
    public void teleopPeriodic() {
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
