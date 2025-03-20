
package frc.robot.seafinder2.commands.retracts;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Core;
import frc.robot.seafinder2.SF2Constants;
import frc.robot.seafinder2.commands.StaticBackCommand;
import frc.robot.seafinder2.commands.limbControl.ManipulatorCommand;
import frc.robot.seafinder2.commands.limbControl.ArmCommand;
import frc.robot.seafinder2.commands.limbControl.ElevatorCommand;
import frc.robot.seafinder2.commands.limbControl.OuttakeCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;

public class RetractL4 extends SequentialCommandGroup {

    CommandSwerveDrivetrain drivetrain;
    ManipulatorSubsystem manipulatorSubsystem;
    ElevatorSubsystem elevatorSubsystem;
    ArmSubsystem armSubsystem;

    public RetractL4(Core core) {
        if (core == null) {throw new IllegalArgumentException("Robot core is null");}

        drivetrain = core.getDrivetrain();
        manipulatorSubsystem = core.getManipulatorSubsystem();
        elevatorSubsystem = core.getElevatorSubsystem();
        armSubsystem = core.getArmSubsystem();

        double elevatorDelta = -Constants.RETRACT_ELEVATOR_DOWNSHIFT;
        double armDelta = -9;
        double outtakeSpeed = 0.3;
        double backDistance = -0.8;
        double backSpeed = -0.7;
        double waitDuration = 0.1;

        Command waitCommand = new WaitCommand(waitDuration);

        Command elevatorCommand = new SequentialCommandGroup(new WaitCommand(0.6), new ElevatorCommand(elevatorSubsystem, elevatorDelta, false)); 
        // TODO: This arm command is now dangerous, appears to be an absolute change, not relative
        Command armCommand = new InstantCommand(() -> armSubsystem.armGoTo(SF2Constants.SETPOINT_REEF_T4.getArm() + armDelta));
        Command outtakeCommand = (new OuttakeCommand(manipulatorSubsystem, outtakeSpeed)).withTimeout(1.0);
        Command scoreCoral = new ParallelCommandGroup(elevatorCommand, armCommand, outtakeCommand);

        Command backCommand = (new StaticBackCommand(drivetrain, backDistance, backSpeed)).withTimeout(0.4);

        Command logStart = new InstantCommand(() -> {System.out.println("RETRACT L4 STARTED");});
        Command logEnd = new InstantCommand(() -> {System.out.println("RETRACT L4 ENDED");});


        this.addCommands(logStart, waitCommand, scoreCoral, backCommand, logEnd);
    }
}
