package frc.robot.seafinder2.commands.limbControl;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class ManipulatorCommand extends Command {
    private ArmSubsystem armSubsystem;
    private double positionArm;
    private double positionWrist;

    public ManipulatorCommand(ArmSubsystem armSubsystem, double valueArm, boolean isPositionArm, double valueWrist, boolean isPositionWrist) {
        this.armSubsystem = armSubsystem;
        if (isPositionArm) {
            this.positionArm = valueArm;
        } else {
            this.positionArm = this.armSubsystem.armMotor2.getPosition().getValueAsDouble() + valueArm;
        }
        if(isPositionWrist){
            this.positionWrist = valueWrist;
        } else {
            this.positionWrist = this.armSubsystem.wristMotor.getPosition().getValueAsDouble()+valueWrist;
        }
        
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
        armSubsystem.armGoTo(positionArm);
        armSubsystem.wristGoTo(positionWrist);
        System.out.println("MANIPULATOR COMMAND STARTED");
    }

    private int clock = 0;

    @Override
    public boolean isFinished() {
        int numDone = 0;

        if (Math.abs(armSubsystem.armMotor2.getPosition().getValueAsDouble() - positionArm) < 30 * (Math.PI / 180.0)) { // Magic number ish - Ask kevin ig
            numDone++;
        } else {
            clock++;
            if (clock >= 15) {
                System.out.println("ARM ERROR: " + ((180.0 / Math.PI) * Math.abs(armSubsystem.armMotor2.getPosition().getValueAsDouble() - positionArm)));
                clock = 0;
            }
        }

        if (Math.abs(armSubsystem.armMotor2.getPosition().getValueAsDouble() - positionWrist) < 260) { // Magic value - Ask kevin ig
            numDone++;
        } else {
            if (clock++ >= 15) {
                System.out.println("WRIST ERROR: " + ((180.0 / Math.PI) * Math.abs(armSubsystem.armMotor2.getPosition().getValueAsDouble() - positionWrist)));
                clock = 0;
            }
        }

        return numDone == 2;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("MANIPULATOR COMMAND ENDED");
    }
}
