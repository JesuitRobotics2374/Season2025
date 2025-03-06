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
import frc.robot.seafinder.commands.TimedForward;
import frc.robot.seafinder.commands.ZeroElevator;
import frc.robot.seafinder.utils.AStar;
import frc.robot.seafinder.utils.Apriltags;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.LimelightObject;

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
        Command seedForAuto = new InstantCommand(() -> m_core.getDrivetrain().seedRobotAuto());

        LimelightObject llRight = Constants.LIMELIGHTS_ON_BOARD[0];
        // Command snapToLimelight = new InstantCommand(() -> m_core.getDrivetrain().alignToVision(llRight, (LimelightHelpers.getBotPoseEstimate_wpiBlue(llRight.name)).pose, true));

        // Command waitForSync = new WaitCommand(3);

        InstantCommand raiseElevator = new InstantCommand(() -> m_core.getElevatorSubsystem().raise(10));
        WaitCommand waitCommand = new WaitCommand(1);
        InstantCommand raiseArm =  new InstantCommand( () -> m_core.getArmSubsystem().armGoTo(9));
        InstantCommand lower_to_limt = new InstantCommand( () -> m_core.getElevatorSubsystem().lower_to_limt() );
        Command goToSetpoint = new InstantCommand(() -> m_core.moveToSetpoint(Constants.SETPOINT_MIN));


        m_core.getPathfinderSubsystem().clearSequence();
        int[][] path = m_core.getNavInterfaceSubsystem().loadPathData();
        System.out.println("Path loaded: " + path.length);
        InstantCommand pathfinder = new InstantCommand(() -> m_core.getPathfinderSubsystem().executePath(path));

        // Command moveForward = new TimedForward(m_core.getDrivetrain(), 1.5);
        
        m_core.autoCommandGroup = new SequentialCommandGroup(seedForAuto, raiseElevator, raiseArm, waitCommand, goToSetpoint); 
        m_core.autoCommandGroup.schedule();


      //  InitRaiseArm moveArm = new InitRaiseArm(m_core.getArmSubsystem());
      //  ZeroElevator zeroElevator = new ZeroElevator(m_core.getElevatorSubsystem());
        
        //SequentialCommandGroup commandGroup = new SequentialCommandGroup(moveArm, zeroElevator, pathfinder)

    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void autonomousExit() {
    }

    @Override
    public void teleopInit() {

    //     System.out.println("Teleop-Iit");
    //     InstantCommand raiseElevator = new InstantCommand(() -> m_core.getElevatorSubsystem().raise(5));
    //     WaitCommand waitCommand = new WaitCommand(0.7);
    //     InitRaiseArm moveArm = new InitRaiseArm(m_core.getArmSubsystem());
    //    // InstantCommand raiseArm =  new InstantCommand( () -> m_core.getArmSubsystem().armGoTo(18.68));
    //     InstantCommand lower_to_limt = new InstantCommand( () -> m_core.getElevatorSubsystem().lower_to_limt() );
    //     SequentialCommandGroup commandGroup = new SequentialCommandGroup(raiseElevator, waitCommand, moveArm, lower_to_limt); 
    //     commandGroup.schedule();
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
