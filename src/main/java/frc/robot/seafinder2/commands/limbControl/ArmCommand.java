package frc.robot.seafinder2.commands.limbControl;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;

public class ArmCommand extends Command {
    private ArmSubsystem armSubsystem;
    private double position;

    public ArmCommand(ArmSubsystem armSubsystem, double value, boolean isPosition) {
        this.armSubsystem = armSubsystem;

        if (isPosition) {
            this.position = value;
        } else {
            this.position = this.armSubsystem.armMotor2.getPosition().getValueAsDouble() + value;
        }
        
        // addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
        armSubsystem.armGoTo(position);
    }

    private int clock = 0;

    @Override
    public boolean isFinished() {
        if (Math.abs(armSubsystem.armMotor2.getPosition().getValueAsDouble() - position) < 30 * (Math.PI / 180.0)) { // Magic number ish - Ask kevin ig
            return true;
        } else {
            clock++;
            if (clock >= 15) {
                System.out.println("ARM ERROR: " + ((180.0 / Math.PI) * Math.abs(armSubsystem.armMotor2.getPosition().getValueAsDouble() - position)));
                clock = 0;
            }
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Init Arm Command Ended");
    }
}
