package frc.robot.seafinder.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;

public class RetractComponents extends Command {

    private CommandSwerveDrivetrain drivetrain;
    private ManipulatorSubsystem manipulatorSubsystem;
    private ElevatorSubsystem elevatorSubsystem;
    private ArmSubsystem armSubsystem;
    private WaitCommand commandEndWait;

    String queuedRetractAction = null;

    public RetractComponents(CommandSwerveDrivetrain drivetrain, ManipulatorSubsystem manipulatorSubsystem,
            ElevatorSubsystem elevatorSubsystem, ArmSubsystem armSubsystem, String retractAction) {
        this.drivetrain = drivetrain;
        this.manipulatorSubsystem = manipulatorSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
        this.armSubsystem = armSubsystem;

        this.queuedRetractAction = retractAction;
    }

    public void initialize() {
        switch (queuedRetractAction) {
            case "none":
                break;
            case "t4": // Tuned on tag 19 right & left - Note at top
                System.out.println("Running retract macro: backAndDown");

                Command waitAndEle = new SequentialCommandGroup(
                        new WaitCommand(0.5),
                        new InstantCommand(
                                () -> elevatorSubsystem.changeBy(-Constants.RETRACT_ELEVATOR_DOWNSHIFT)));

                Command waitAndOuttake = new SequentialCommandGroup(
                        new WaitCommand(0.75),
                        new InstantCommand(() -> manipulatorSubsystem.outtake(0.1)),
                        new WaitCommand(0.3),
                        new InstantCommand(() -> manipulatorSubsystem.outtake(0.25)),
                        new WaitCommand(0.3),
                        new InstantCommand(() -> manipulatorSubsystem.stop()));

                Command waitAndBack = new SequentialCommandGroup(
                        new WaitCommand(3),
                        new StaticBackCommand(drivetrain, Constants.STATIC_BACK_TIME, -0.4));

                Command lowerArm = new InstantCommand(() -> armSubsystem.armChangeBy(-9));

                ParallelCommandGroup result4 = new ParallelCommandGroup(waitAndEle, waitAndOuttake, lowerArm,
                        waitAndBack);

                result4.schedule();

                break;
            case "t3": // Tuned on tag 19 right & left - Note at top
                System.out.println("Running retract macro: backAndDown");

                Command waitAndOuttake3 = new SequentialCommandGroup(
                        new InstantCommand(() -> manipulatorSubsystem.outtake(0.1)),
                        new WaitCommand(0.2),
                        new InstantCommand(() -> manipulatorSubsystem.stop()),
                        new WaitCommand(0.1),
                        new InstantCommand(() -> manipulatorSubsystem.outtake(0.1)),
                        new WaitCommand(0.3),
                        new InstantCommand(() -> manipulatorSubsystem.stop()));

                Command waitAndBack3 = new SequentialCommandGroup(new WaitCommand(0.8),
                        new StaticBackCommand(drivetrain, Constants.STATIC_BACK_TIME, -1));

                Command elevatorDown = new InstantCommand(() -> elevatorSubsystem.changeBy(-50));

                Command lowerArm3 = new InstantCommand(() -> armSubsystem.armChangeBy(-9));

                ParallelCommandGroup result3 = new ParallelCommandGroup(waitAndOuttake3, waitAndBack3, elevatorDown,
                        lowerArm3);

                result3.schedule();

                break;
            case "t2": // Tuned on tage 19 right & left - Note at top
                System.out.println("Running retract macro: backAndDown");

                SequentialCommandGroup waitAndOuttake2 = new SequentialCommandGroup(
                        new WaitCommand(1.0),
                        new InstantCommand(() -> manipulatorSubsystem.outtake(0.4)),
                        new WaitCommand(3.2),
                        new InstantCommand(() -> manipulatorSubsystem.stop()));

                SequentialCommandGroup waitAndBack2 = new SequentialCommandGroup(
                        new WaitCommand(1.7),
                        new StaticBackCommand(drivetrain, Constants.STATIC_BACK_TIME, -1));

                Command elevatorDown2 = new InstantCommand(() -> elevatorSubsystem.changeBy(-11));

                Command lowerArm2 = new InstantCommand(() -> armSubsystem.armChangeBy(-9));

                ParallelCommandGroup result2 = new ParallelCommandGroup(waitAndOuttake2, waitAndBack2, elevatorDown2, lowerArm2);

                result2.schedule();
                
                break;
            default:
                break;
        }

        end(false);
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Retract Command Schedule ");
    }
}
