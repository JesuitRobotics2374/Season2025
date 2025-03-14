
package frc.robot.seafinder2.commands.retracts;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Core;
import frc.robot.seafinder2.commands.StaticBackCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;

public class RetractL3 extends Command {

    CommandSwerveDrivetrain drivetrain;
    ManipulatorSubsystem manipulatorSubsystem;
    ElevatorSubsystem elevatorSubsystem;
    ArmSubsystem armSubsystem;

    public RetractL3(Core core) {
        if (core == null) {
            throw new IllegalArgumentException("Robot core is null");
        }
        drivetrain = core.getDrivetrain();
        manipulatorSubsystem = core.getManipulatorSubsystem();
        elevatorSubsystem = core.getElevatorSubsystem();
        armSubsystem = core.getArmSubsystem();
    }

    public void initialize() {
        System.out.println("RETRACT L3 STARTED");

        SequentialCommandGroup waitAndBack = new SequentialCommandGroup(
                new WaitCommand(1.4),
                (new StaticBackCommand(drivetrain, -0.4, -1)).withTimeout(1.5));
        
        manipulatorSubsystem.spinIntakeCommand(0.4).withTimeout(3.2).schedule();
        elevatorSubsystem.changeBy(-30);
        armSubsystem.armChangeBy(-9);
        waitAndBack.schedule();
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("RETRACT L3 ENDED");
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
