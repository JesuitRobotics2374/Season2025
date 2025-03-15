package frc.robot.seafinder2.commands.limbControl;

import com.ctre.phoenix6.hardware.core.CoreCANrange;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ManipulatorSubsystem;

public class IntakeCommand extends Command {

    private ManipulatorSubsystem manipulatorSubsystem;
    private CoreCANrange sensor;
    private int clock;
    private boolean isIntaking;
    

    public IntakeCommand(ManipulatorSubsystem manipulatorSubsystem){
        this.manipulatorSubsystem = manipulatorSubsystem;
        this.sensor = manipulatorSubsystem.sensor;
    }
    
    @Override
    public void initialize() {
        System.out.println("INTAKE COMMAND START");
        isIntaking= true;
    }

    public void periodic() {
        
        boolean withinRange = sensor.getDistance().getValueAsDouble() <= 0.06
                && sensor.getIsDetected().getValueAsDouble() == 1.0;
        
        manipulatorSubsystem.spinAt(-.75);

        if (clock > 10 && withinRange) {
            clock = 5;
        }

        if (isIntaking && withinRange) {
            manipulatorSubsystem.stop();
            isIntaking = false;
            end(false);
        }
        
    }


    @Override
    public void end(boolean interrupted) {
        System.out.println("Init Intake Command Ended");
    }
}
