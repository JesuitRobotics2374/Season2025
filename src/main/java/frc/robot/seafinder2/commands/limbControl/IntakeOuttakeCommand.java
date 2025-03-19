package frc.robot.seafinder2.commands.limbControl;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;

public class IntakeOuttakeCommand extends Command {
    private ManipulatorSubsystem manipulatorSubsystem;
    private double speed;

    public IntakeOuttakeCommand(ManipulatorSubsystem manipulatorSubsystem, double speed) {
        this.manipulatorSubsystem = manipulatorSubsystem;
        this.speed = speed;
    }

    @Override
    public void initialize() {
        System.out.println("INTAKE-OUTTAKE COMMAND START");
        this.manipulatorSubsystem.setOverride(true);
        this.manipulatorSubsystem.outtake(speed);
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("INTAKE-OUTTAKE COMMAND END");
        this.manipulatorSubsystem.stopOuttake();
    }
}
