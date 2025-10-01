package frc.robot.seafinder2.commands.limbControl;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;

public class ElevatorCommand extends Command {
    private ElevatorSubsystem elevatorSubsystem;
    private ManipulatorSubsystem manipulatorSubsystem;
    private double position;
    private boolean isPosition;

    public ElevatorCommand(ElevatorSubsystem elevatorSubsystem, ManipulatorSubsystem manipulatorSubsystem, double value, boolean isPosition) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.manipulatorSubsystem = manipulatorSubsystem;
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
        elevatorSubsystem.elevatorGoToDouble(manipulatorSubsystem, position);
    }

    private int clock = 0;

    @Override
    public boolean isFinished() {
       //  if (Math.abs(elevatorSubsystem.elevatorMotor1.getPosition().getValueAsDouble() - position) < ((isPosition) ? 0.3 : 13.00)) { // Magic number sorrryyyy - ask kevin ig

        double current_position = elevatorSubsystem.elevatorMotor1.getPosition().getValueAsDouble();
        double diff = current_position - position;


         if (position == 1.0 && diff < 0.3) {   //NW  If the postiton was negative it would result in a number between 0.3 an 1  usually 0.9  this would cause the command to never finisj
            return true;
        } 

        diff = Math.abs(diff);
       
        if (diff < (isPosition ? 0.3 : 13.00) ) { // Magic number sorrryyyy - ask kevin ig    
            return true;
        } else {
            clock++;
            if (clock >= 15) {
               // System.out.println("ELEVATOR COMMAND ERROR: " + Math.abs(elevatorSubsystem.elevatorMotor1.getPosition().getValueAsDouble() - position));
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