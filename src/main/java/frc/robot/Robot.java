// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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

        System.out.println("Auto-Iit");
       // InstantCommand setElevatorZero = new InstantCommand(() -> m_core.getElevatorSubsystem().zeroElevator());
        // setElevatorZero.schedule();
        InstantCommand raiseElevator = new InstantCommand(() -> m_core.getElevatorSubsystem().raise(4));
        raiseElevator.schedule();
   
        WaitCommand waitCommand = new WaitCommand(0.3);
        waitCommand.schedule();


       // InitRaiseArm moveArm = new InitRaiseArm(m_core.getArmSubsystem());

        InstantCommand raiseArm =  new InstantCommand( () -> m_core.getArmSubsystem().armGoTo(18.68));
        raiseArm.schedule();
     //   ZeroElevator zeroElevator = new ZeroElevator(m_core.getElevatorSubsystem());
     //   zeroElevator.schedule();

        System.out.println("lower to limit");
        InstantCommand lower_to_limt = new InstantCommand( () -> m_core.getElevatorSubsystem().lower_to_limt() );
        lower_to_limt.schedule();
        

        System.out.println("auto_init done");
      //  SequentialCommandGroup commandGroup = new SequentialCommandGroup(raiseArm, zeroElevator);
      //  commandGroup.schedule();

/* 
        m_core.getPathfinderSubsystem().clearSequence();
        int[][] path = m_core.getNavInterfaceSubsystem().loadPathData();
        System.out.println("Path loaded: " + path.length);
        InstantCommand pathfinder = new InstantCommand(() -> m_core.getPathfinderSubsystem().executePath(path));
        
        InitRaiseArm moveArm = new InitRaiseArm(m_core.getArmSubsystem());
        ZeroElevator zeroElevator = new ZeroElevator(m_core.getElevatorSubsystem());
        
        SequentialCommandGroup commandGroup = new SequentialCommandGroup(moveArm, zeroElevator, pathfinder);
        commandGroup.schedule();

*/
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void autonomousExit() {
    }

    @Override
    public void teleopInit() {
        InstantCommand raiseElevator = new InstantCommand(() -> m_core.getElevatorSubsystem().raise(4));
        raiseElevator.schedule();

        WaitCommand waitCommand = new WaitCommand(0.3);
        waitCommand.schedule();

        // InstantCommand setElevatorZero = new InstantCommand(() -> m_core.getElevatorSubsystem().zeroElevator());
        // setElevatorZero.schedule();
        
        InitRaiseArm moveArm = new InitRaiseArm(m_core.getArmSubsystem());
        ZeroElevator zeroElevator = new ZeroElevator(m_core.getElevatorSubsystem());
        
        SequentialCommandGroup commandGroup = new SequentialCommandGroup(moveArm, zeroElevator);
        commandGroup.schedule();
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
