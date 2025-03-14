
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

public class RetractL2 extends Command {

    CommandSwerveDrivetrain drivetrain;
    ManipulatorSubsystem manipulatorSubsystem;
    ElevatorSubsystem elevatorSubsystem;
    ArmSubsystem armSubsystem;

    public RetractL2(Core core) {
        if (core == null) {
            throw new IllegalArgumentException("Robot core is null");
        }
        drivetrain = core.getDrivetrain();
        manipulatorSubsystem = core.getManipulatorSubsystem();
        elevatorSubsystem = core.getElevatorSubsystem();
        armSubsystem = core.getArmSubsystem();
    }

    public void initialize() {
        System.out.println("RETRACT L2 STARTED");

        SequentialCommandGroup waitAndOuttake = new SequentialCommandGroup(
                new WaitCommand(1.0),
                manipulatorSubsystem.spinIntakeCommand(0.4).withTimeout(3.2));
        SequentialCommandGroup waitAndBack = new SequentialCommandGroup(
                new WaitCommand(1.7),
                (new StaticBackCommand(drivetrain, -0.4, -1)).withTimeout(1.5));

        elevatorSubsystem.changeBy(-11);
        armSubsystem.armChangeBy(-9);
        waitAndOuttake.schedule();
        waitAndBack.schedule();
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("RETRACT L2 ENDED");
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
