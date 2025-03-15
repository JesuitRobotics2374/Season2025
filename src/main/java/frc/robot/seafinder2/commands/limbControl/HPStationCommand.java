package frc.robot.seafinder2.commands.limbControl;

import edu.wpi.first.math.spline.PoseWithCurvature;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;

public class HPStationCommand extends Command {
    private ArmSubsystem armSubsystem;
    private double positionArm;
    private double positionWrist;

    public HPStationCommand(ArmSubsystem armSubsystem, double valueArm, boolean isPositionArm, double valueWrist, boolean isPositionWrist) {
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
        System.out.println("wristGoToRun: " +positionWrist);
    }

    private int clock = 0;

    @Override
    public boolean isFinished() {
        if (Math.abs(armSubsystem.armMotor2.getPosition().getValueAsDouble() - positionArm) < 30 * (Math.PI / 180.0)) { // Magic number ish - Ask kevin ig
            return true;
        } else {
            clock++;
            if (clock >= 15) {
                System.out.println("ARM ERROR: " + ((180.0 / Math.PI) * Math.abs(armSubsystem.armMotor2.getPosition().getValueAsDouble() - positionArm)));
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
