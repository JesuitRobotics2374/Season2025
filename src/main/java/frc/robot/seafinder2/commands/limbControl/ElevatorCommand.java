package frc.robot.seafinder2.commands.limbControl;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorCommand extends Command {
    private ElevatorSubsystem elevatorSubsystem;
    private double position;
    private boolean isPosition;

    public ElevatorCommand(ElevatorSubsystem elevatorSubsystem, double value, boolean isPosition) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.isPosition = isPosition;

        if (this.isPosition) {
            this.position = value;
        } else {
            this.position = this.elevatorSubsystem.elevatorMotor1.getPosition().getValueAsDouble() + value;
        }
        
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {
        elevatorSubsystem.elevatorGoToDouble(position);
    }

    private int clock = 0;

    @Override
    public boolean isFinished() {
        if (Math.abs(elevatorSubsystem.elevatorMotor1.getPosition().getValueAsDouble() - position) < ((isPosition) ? 0.3 : 13.00)) { // Magic number sorrryyyy - ask kevin ig
            return true;
        } else {
            clock++;
            if (clock >= 15) {
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
