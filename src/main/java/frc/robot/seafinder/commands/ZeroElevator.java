package frc.robot.seafinder.commands;

import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class ZeroElevator extends Command {
    private ElevatorSubsystem elevatorSubsytem;

    public ZeroElevator(ElevatorSubsystem elevatorSubsystem) {
        this.elevatorSubsytem = elevatorSubsystem;
        
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {
        elevatorSubsytem.zeroElevator();
    }

    @Override
    public boolean isFinished() {
        return !elevatorSubsytem.limitSwitch.get();
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Init Zeroing Elevator Ended");
    }
}
