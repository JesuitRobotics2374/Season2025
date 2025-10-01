// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.seafinder2.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.seafinder2.SF2Constants;
import frc.robot.seafinder2.commands.limbControl.EjectCommand;
import frc.robot.seafinder2.commands.limbControl.ElevatorCommand;
import frc.robot.seafinder2.utils.Target.Setpoint;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;

public class ScoreCommand extends SequentialCommandGroup {

  public ScoreCommand(Setpoint setpoint, ElevatorSubsystem elevatorSubsystem, ManipulatorSubsystem manipulatorSubsystem) {

    addRequirements(elevatorSubsystem,manipulatorSubsystem);

    addCommands(
        new ElevatorCommand(elevatorSubsystem, manipulatorSubsystem, setpoint.getElevator(), true),
            new EjectCommand(manipulatorSubsystem),
            new ElevatorCommand(elevatorSubsystem, manipulatorSubsystem, SF2Constants.SETPOINT_MIN.getElevator(), true)
    );
  }

}
