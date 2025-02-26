package frc.robot.seafinder.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;

public class InitRaiseArm extends Command {
    private ArmSubsystem armSubsystem;
    private double goal;

    public InitRaiseArm(ArmSubsystem armSubsystem, double goal) {
        this.armSubsystem = armSubsystem;
        this.goal = goal;
        
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
        if (!armSubsystem.armPassedGoal()) {
            armSubsystem.armGoTo(goal);
        }
        // else {
        //     armSubsystem.setGoalToCurrent():
        // }
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
