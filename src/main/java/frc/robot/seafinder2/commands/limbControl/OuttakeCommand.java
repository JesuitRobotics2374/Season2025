package frc.robot.seafinder2.commands.limbControl;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;

public class OuttakeCommand extends Command {
    private ManipulatorSubsystem manipulatorSubsystem;
    private double speed;

    public OuttakeCommand(ManipulatorSubsystem manipulatorSubsystem, double speed) {
        this.manipulatorSubsystem = manipulatorSubsystem;
        this.speed = speed;
    }

    @Override
    public void initialize() {
        System.out.println("OUTTAKE COMMAND START");
        this.manipulatorSubsystem.outtake(speed);
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("OUTTAKE COMMAND END");
        this.manipulatorSubsystem.stopOuttake();
    }
}
