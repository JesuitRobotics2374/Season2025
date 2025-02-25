package frc.robot.seafinder.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;

public class InitRaiseArm extends Command {
    private ArmSubsystem armSubsystem;
    private double addition = 0.0;

    public InitRaiseArm(ArmSubsystem armSubsystem) {
        this.armSubsystem = armSubsystem;
        
        addRequirements(armSubsystem);
    }

    public InitRaiseArm(ArmSubsystem armSubsystem, boolean notUsed) {
        this.armSubsystem = armSubsystem;
        addition = 10.0;
        
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
        armSubsystem.armGoTo(Constants.ARM_HORIZONTAL + addition);
    }

    @Override
    public boolean isFinished() {
        return armSubsystem.armPassedGoal();
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Init Raise Arm Ended");
    }
}
