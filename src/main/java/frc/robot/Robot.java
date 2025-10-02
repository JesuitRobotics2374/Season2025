// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import org.photonvision.EstimatedRobotPose;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.seafinder2.SF2Constants;
import frc.robot.seafinder2.commands.CanRangeStation;
import frc.robot.seafinder2.commands.ExactAlign;
import frc.robot.seafinder2.commands.ScoreCommand;
import frc.robot.seafinder2.commands.limbControl.ElevatorCommand;
import frc.robot.seafinder2.utils.Apriltags;
import frc.robot.seafinder2.utils.Target;
import frc.robot.seafinder2.utils.Target.Height;
import frc.robot.seafinder2.utils.Target.Landmark;
import frc.robot.seafinder2.utils.Target.Location;
import frc.robot.seafinder2.utils.Target.Side;
import frc.robot.subsystems.VisionSubsystem;

public class Robot extends TimedRobot {

    private final Core m_core;

    private Target target1;
    private Target target2;

    public Robot() {



        m_core = new Core();
        Apriltags.loadField();

        m_core.getDrivetrain().seedRobotAuto();

        PathfindingCommand.warmupCommand().schedule();

        VisionSubsystem.initializeVisionSubsystem();


        target1 = new Target(m_core);
        target1.setLocation(new Target.Location(Landmark.REEF_BACK, Side.RIGHT));
        target1.setHeight(Target.Height.BRANCH_L4); // This is a structural requirement, but we don't use it here.

        target2 = new Target(m_core);
        target2.setLocation(new Target.Location(Landmark.REEF_FRONT_RIGHT, Side.RIGHT));
        target2.setHeight(Target.Height.BRANCH_L4); // This is a structural requirement, but we don't use it here.


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
      ///  m_core.getDrivetrain().configNeutralMode(NeutralModeValue.Brake);
       // m_core.getElevatorSubsystem().stopElevator();
       m_core.getDrivetrain().configNeutralMode(NeutralModeValue.Coast); 

        System.out.println("Auto-Iit");
       m_core.getDrivetrain().seedRobotAuto();
/* 
        List<EstimatedRobotPose> estimatedRobotPoses = VisionSubsystem.getGlobalFieldPoses();

        for (EstimatedRobotPose estimatedRobotPose : estimatedRobotPoses) {
            if (estimatedRobotPose != null) {
                // field.getObject("Vision" + displayCounter).setPose(fp.pose);
                m_core.getDrivetrain().alignToVision(estimatedRobotPose, false);
            }
        } */

      //  m_core.getElevatorSubsystem().doEstimatedZero();

       // Command raiseElevator = new ElevatorCommand(m_core.getElevatorSubsystem(), 1, false);

        // First Auto
        m_core.pathfinderSubsystem.queueFind(new Location(Landmark.REEF_BACK, Side.LEFT), true);
        m_core.pathfinderSubsystem.queueAlign(Height.BRANCH_L3);
        // score first auto
        Command a2 = (new ScoreCommand(SF2Constants.SETPOINT_REEF_T4, m_core.elevatorSubsystem, m_core.manipulatorSubsystem));
         m_core.pathfinderSubsystem.autoSequence.addCommands(a2);


         
        // Human Station
        m_core.pathfinderSubsystem.queueFind(new Location(Landmark.STATION_RIGHT)); 
        m_core.pathfinderSubsystem.autoSequence.addCommands(new CanRangeStation(m_core.getDrivetrain()));
        m_core.pathfinderSubsystem.autoSequence.addCommands(m_core.getManipulatorSubsystem().load());
     

        //Second piece
        m_core.pathfinderSubsystem.queueFind(new Location(Landmark.REEF_FRONT_RIGHT));

        Command a3 = (new ScoreCommand(SF2Constants.SETPOINT_REEF_T3, m_core.elevatorSubsystem, m_core.manipulatorSubsystem));
        m_core.pathfinderSubsystem.autoSequence.addCommands(a3);


        m_core.getPathfinderSubsystem().autoSequence.schedule();

        System.out.println("Auto schedule complete");

    
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void autonomousExit() {
        m_core.getDrivetrain().configNeutralMode(NeutralModeValue.Brake);
        m_core.getElevatorSubsystem().stopElevator();
    }

    @Override
    public void teleopInit() {
        m_core.getDrivetrain().configNeutralMode(NeutralModeValue.Brake);
        m_core.getManipulatorSubsystem().stop();
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
        m_core.getElevatorSubsystem().stopElevator();
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
