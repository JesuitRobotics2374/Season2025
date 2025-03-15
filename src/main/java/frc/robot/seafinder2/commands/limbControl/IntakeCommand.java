package frc.robot.seafinder2.commands.limbControl;

import com.ctre.phoenix6.hardware.core.CoreCANrange;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ManipulatorSubsystem;

public class IntakeCommand extends Command {

    private ManipulatorSubsystem manipulatorSubsystem;
    private CoreCANrange sensor;
    private int clock;

    public IntakeCommand(ManipulatorSubsystem manipulatorSubsystem) {
        this.manipulatorSubsystem = manipulatorSubsystem;
        this.sensor = manipulatorSubsystem.sensor;
    }

    @Override
    public void initialize() {

        System.out.println("Intake command started");

        clock = 0;
        manipulatorSubsystem.spinAt(-.75);
    }

    @Override
    public void execute() {

        boolean withinRange = sensor.getDistance().getValueAsDouble() <= 0.06
                && sensor.getIsDetected().getValueAsDouble() == 1.0;

        if (withinRange) {
            clock++;
        } else {
            clock = 0;
        }

        if (withinRange && clock == 7) {
            manipulatorSubsystem.stop();
            end(false);
        }

    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Intake Command Ended");
    }
}
