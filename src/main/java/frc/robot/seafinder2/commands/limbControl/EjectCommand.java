package frc.robot.seafinder2.commands.limbControl;

import com.ctre.phoenix6.hardware.core.CoreCANrange;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ManipulatorSubsystem;

public class EjectCommand extends Command {

    private ManipulatorSubsystem manipulatorSubsystem;
    private int clock;

    boolean done = false;

    public EjectCommand(ManipulatorSubsystem manipulatorSubsystem) {
        this.manipulatorSubsystem = manipulatorSubsystem;
    }

    @Override
    public void initialize() {

        done = false;

        System.out.println("Intake command started");

        clock = 0;
    }

    @Override
    public void execute() {

        manipulatorSubsystem.feed();

        clock++;

        if (clock == 20) {
            manipulatorSubsystem.stop();
            done = true;
        }

    }

    @Override
    public void end(boolean interrupted) {
        manipulatorSubsystem.clearLoaded();
        System.out.println("Intake Command Ended");
    }

    @Override
    public boolean isFinished() {
        return done;
    }
}
