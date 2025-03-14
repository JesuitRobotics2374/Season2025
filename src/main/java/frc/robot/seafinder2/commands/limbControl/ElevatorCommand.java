package frc.robot.seafinder2.commands.limbControl;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorCommand extends Command {
    private ElevatorSubsystem elevatorSubsystem;
    private double position;

    public ElevatorCommand(ElevatorSubsystem elevatorSubsystem, double position) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.position = position;
        
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {
        elevatorSubsystem.elevatorGoToDouble(position);
    }

    private int clock = 0;

    @Override
    public boolean isFinished() {
        if (Math.abs(elevatorSubsystem.elevatorMotor1.getPosition().getValueAsDouble() - position) < 13.00) { // Magic number sorrryyyy
            return true;
        } else {
            clock++;
            if (clock >= 20) {
                System.out.println("ELEVATOR COMMAND ERROR: " + Math.abs(elevatorSubsystem.elevatorMotor1.getPosition().getValueAsDouble() - position));
                clock = 0;
            }
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Init Elevator Command Ended");
    }
}
