
package frc.robot.seafinder2.commands.retracts;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Core;
import frc.robot.seafinder2.commands.StaticBackCommand;
import frc.robot.seafinder2.commands.limbControl.HPStationCommand;
import frc.robot.seafinder2.commands.limbControl.ArmCommand;
import frc.robot.seafinder2.commands.limbControl.ElevatorCommand;
import frc.robot.seafinder2.commands.limbControl.OuttakeCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;

public class RetractL4 extends Command {

    CommandSwerveDrivetrain drivetrain;
    ManipulatorSubsystem manipulatorSubsystem;
    ElevatorSubsystem elevatorSubsystem;
    ArmSubsystem armSubsystem;

    public RetractL4(Core core) {
        if (core == null) {
            throw new IllegalArgumentException("Robot core is null");
        }
        drivetrain = core.getDrivetrain();
        manipulatorSubsystem = core.getManipulatorSubsystem();
        elevatorSubsystem = core.getElevatorSubsystem();
        armSubsystem = core.getArmSubsystem();
    }

    public void initialize() {
        System.out.println("RETRACT L4 STARTED");

        double elevatorDelta = -Constants.RETRACT_ELEVATOR_DOWNSHIFT;
        double armDelta = -9;
        double outtakeSpeed = 1.0;
        double backDistance = -0.6;
        double backSpeed = -0.6;

        Command elevatorCommand = new ElevatorCommand(elevatorSubsystem, elevatorDelta, false);
        Command armCommand = new ArmCommand(armSubsystem, armDelta, false);
        Command scoreCoral = new ParallelCommandGroup(elevatorCommand, armCommand);

        Command outtakeCommand = (new OuttakeCommand(manipulatorSubsystem, outtakeSpeed)).withTimeout(3.0);
        Command backCommand = (new StaticBackCommand(drivetrain, backDistance, backSpeed)).withTimeout(1.0);
        Command removeRobot = new ParallelCommandGroup(outtakeCommand, backCommand);

        SequentialCommandGroup commandGroup = new SequentialCommandGroup(scoreCoral, removeRobot);
        commandGroup.schedule();
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("RETRACT L4 STARTED");
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
