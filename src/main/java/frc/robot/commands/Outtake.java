package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.Constants;

public class Outtake extends SequentialCommandGroup {

    public Outtake(ManipulatorSubsystem manipulatorSubsystem) {

        addRequirements(manipulatorSubsystem);
        
        InstantCommand start = new InstantCommand(() -> manipulatorSubsystem.outtake());
        InstantCommand stop = new InstantCommand(() -> manipulatorSubsystem.stopEject());

        SequentialCommandGroup outtake = new SequentialCommandGroup(start, new WaitCommand(Constants.outtakeWaitTime), stop);

        outtake.schedule();
    }
    
}
