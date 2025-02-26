// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.ArrayList;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.Pathfinder;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.seafinder.PathfinderSubsystem;
import frc.robot.seafinder.PathfinderSubsystem.Alignment;
import frc.robot.seafinder.commands.ExactAlignRot;
import frc.robot.seafinder.commands.StaticBackCommand;
import frc.robot.seafinder.commands.StationAlign;
import frc.robot.seafinder.interfaces.NavInterfaceSubsystem;
import frc.robot.seafinder.interfaces.PanelSubsystem;
import frc.robot.seafinder.utils.Setpoint;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.TunerConstants;

public class Core {

    public double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * Constants.MAX_SPEED_LOW_ELEVATOR; // kSpeedAt12Volts
                                                                                                       // desired top
                                                                                                       // speed
    public double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond) * Constants.MAX_ANGULAR_RATE; // 3/4
                                                                                                                  // of
                                                                                                                  // a
                                                                                                                  // rotation
                                                                                                                  // //
                                                                                                                  // per
                                                                                                                  // second
                                                                                                                  // max
                                                                                                                  // angular
                                                                                                                  // velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.01).withRotationalDeadband(MaxAngularRate * 0.002) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    // private final SwerveRequest.SwerveDriveBrake brake = new
    // SwerveRequest.SwerveDriveBrake();
    // private final SwerveRequest.PointWheelsAt point = new
    // SwerveRequest.PointWheelsAt();

    // private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driveController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
    public final ManipulatorSubsystem manipulatorSubsystem = new ManipulatorSubsystem();
    public final ArmSubsystem armSubsystem = new ArmSubsystem();

    public final PathfinderSubsystem pathfinderSubsystem = new PathfinderSubsystem(this);

    public final PanelSubsystem panelSubsystem = new PanelSubsystem(pathfinderSubsystem);
    public final NavInterfaceSubsystem navInterfaceSubsystem = new NavInterfaceSubsystem();

    // private final SendableChooser<Command> autoChooser;

    private Command pathfindingCommand;

    private String queuedRetractAction;

    public Core() {
        registerAutoCommands();
        // autoChooser = AutoBuilder.buildAutoChooser();
        configureBindings();
        configureShuffleBoard();

        CameraServer.startAutomaticCapture();

        // drivetrain.setRobotPose(new Pose2d(7.5, 1.5, new Rotation2d(180 * (Math.PI /
        // 180))));
    }

    // A setpoint is a "macro" state. Find its definition in utils folder.
    public void moveToSetpoint(Setpoint setpoint) {
        queuedRetractAction = setpoint.getRetractAction(); // Store what we just did for when we retract
        elevatorSubsystem.elevatorGoToDouble(setpoint.getElevator());
        armSubsystem.armGoTo(setpoint.getArm());
        armSubsystem.wristGoTo(setpoint.getWrist());
    }

    // Based on the last setpoint we aligned to, retract using a very specific set
    // of hardward movements
    public void performRetract() {
        // NOTE: Only tag 19 is secured to reef so only testing happened on tag 19
        if (queuedRetractAction != null) {
            switch (queuedRetractAction) {
                case "none":
                    break;
                case "t4": // Tuned on tag 19 right & left - Note at top
                    System.out.println("Running retract macro: backAndDown");
                    SequentialCommandGroup waitAndEle = new SequentialCommandGroup(
                            new WaitCommand(0.5),
                            new InstantCommand(
                                    () -> elevatorSubsystem.changeBy(-Constants.RETRACT_ELEVATOR_DOWNSHIFT)));
                    SequentialCommandGroup waitAndOuttake = new SequentialCommandGroup(
                            new WaitCommand(0.75),
                            new InstantCommand(() -> manipulatorSubsystem.outtake(0.1)),
                            new WaitCommand(0.3),
                            new InstantCommand(() -> manipulatorSubsystem.outtake(0.25)),
                            new WaitCommand(0.3),
                            new InstantCommand(() -> manipulatorSubsystem.stop()));
                    SequentialCommandGroup waitAndBack = new SequentialCommandGroup(
                            new WaitCommand(3),
                            new StaticBackCommand(drivetrain, -0.2, -0.4));

                    waitAndEle.schedule();
                    waitAndOuttake.schedule();
                    armSubsystem.armChangeBy(-9);
                    waitAndBack.schedule();
                    break;
                case "t3": // Tuned on tag 19 right & left - Note at top
                    System.out.println("Running retract macro: backAndDown");
                    SequentialCommandGroup waitAndOuttake3 = new SequentialCommandGroup(
                            new InstantCommand(() -> manipulatorSubsystem.outtake(0.1)),
                            new WaitCommand(0.2),
                            new InstantCommand(() -> manipulatorSubsystem.stop()),
                            new WaitCommand(0.1),
                            new InstantCommand(() -> manipulatorSubsystem.outtake(0.1)),
                            new WaitCommand(0.3),
                            new InstantCommand(() -> manipulatorSubsystem.stop()));
                    SequentialCommandGroup waitAndBack3 = new SequentialCommandGroup(new WaitCommand(0.8),
                            new StaticBackCommand(drivetrain, -0.2, -1));

                    elevatorSubsystem.changeBy(-50);
                    armSubsystem.armChangeBy(-19);
                    waitAndOuttake3.schedule();
                    waitAndBack3.schedule();
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
                            new StaticBackCommand(drivetrain, -0.2, -1));

                    elevatorSubsystem.changeBy(-11);
                    armSubsystem.armChangeBy(-15);
                    waitAndOuttake2.schedule();
                    waitAndBack2.schedule();
                    break;
                default:
                    break;
            }
        }
        return;
    }

    public void registerAutoCommands() {
        // NamedCommands.registerCommand("OuttakeCommand", new
        // Outtake(outtakeSubsystem));
        // NamedCommands.registerCommand("Test Pathfind", new PathfindBasic(drivetrain,
        // Constants.TEST_PATHFIND_TARGET));

        // PathfindingCommand.warmupCommand().schedule();
    }

    public void configureShuffleBoard() {

        ShuffleboardTab tab = Shuffleboard.getTab("Test");

        // Limelight
        // HttpCamera httpCamera = new HttpCamera("Limelight",
        // "http://limelight.local:5800");
        // CameraServer.addCamera(httpCamera);
        // tab.add(httpCamera).withPosition(7, 0).withSize(3, 2);

        // New List Layout
        // ShuffleboardContainer pos = tab.getLayout("Position", "List
        // Layout").withPosition(0, 0).withSize(2, 3);

        // Field
        tab.add(drivetrain.getField()).withPosition(2, 1).withSize(5, 3);

        // Modes
        // tab.addBoolean("Slow Mode", () -> isSlow()).withPosition(2, 0).withSize(2,
        // 1);
        // tab.addBoolean("Roll Mode", () -> isRoll()).withPosition(5, 0).withSize(1,
        // 1);

        // Robot (Reverse order for list layout)
        // pos.addDouble("Robot R", () -> drivetrain.getRobotR())
        // .withWidget("Gyro");
        // ;
        tab.addDouble("Robot Y", () -> drivetrain.getRobotY());
        // .withWidget("Number Bar");
        tab.addDouble("Robot X", () -> drivetrain.getRobotX());

        // tab.add("Auto Chooser", autoChooser);

    }

    private void configureBindings() {

        // STICK MOVEMENT
        drivetrain.setDefaultCommand(
                drivetrain.applyRequest(() -> drive
                        .withVelocityX(-driveController.getLeftY() * MaxSpeed * getAxisMovementScale())
                        .withVelocityY(-driveController.getLeftX() * MaxSpeed * getAxisMovementScale())
                        .withRotationalRate(-driveController.getRightX() * MaxAngularRate * getAxisMovementScale())));

        driveController.back().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric())); // RESET POSE

        driveController.a().onTrue(drivetrain.runOnce(() -> moveToSetpoint(Constants.SETPOINT_ALGAE_T2))); // RESET POSE
        driveController.b().onTrue(drivetrain.runOnce(() -> moveToSetpoint(Constants.SETPOINT_ALGAE_T3))); // RESET POSE
        driveController.x().onTrue(armSubsystem.runOnce(() -> {armSubsystem.setZero();}));

        // driveController.povDown().onTrue(new InstantCommand(() -> moveToSetpoint(Constants.SETPOINT_ALGAE_T2)));
        // driveController.povLeft().onTrue(new InstantCommand(() -> moveToSetpoint(Constants.SETPOINT_ALGAE_T3)));
        // driveController.povUp().onTrue(new InstantCommand(() -> moveToSetpoint(Constants.SETPOINT_PROCESSOR)));
        // driveController.povRight().onTrue(new InstantCommand(() -> moveToSetpoint(Constants.SETPOINT_BARGE)));

        driveController.leftBumper().whileTrue(elevatorSubsystem.runOnce(() ->
        elevatorSubsystem.lower()));
        driveController.rightBumper().whileTrue(elevatorSubsystem.runOnce(() ->
        elevatorSubsystem.raise()));

        operatorController.rightBumper().onTrue(armSubsystem.runOnce(() ->
        armSubsystem.armUp()));
        operatorController.leftBumper().onTrue(armSubsystem.runOnce(() ->
        armSubsystem.armDown()));

        operatorController.y().onTrue(new InstantCommand(() -> moveToSetpoint(Constants.SETPOINT_BARGE)));
        operatorController.b().onTrue(new InstantCommand(() -> moveToSetpoint(Constants.SETPOINT_HP_INTAKE)));
        operatorController.a().onTrue(new InstantCommand(() -> moveToSetpoint(Constants.SETPOINT_MIN)));
        operatorController.x().onTrue(new InstantCommand(() -> performRetract()));

        // operatorController.back().onTrue(armSubsystem.runOnce(() ->
        // armSubsystem.rotateWristIntake()));
        operatorController.start().onTrue(manipulatorSubsystem.runOnce(() ->
        manipulatorSubsystem.holdAlgae()));

        operatorController.povDown().onTrue(new InstantCommand(() -> moveToSetpoint(Constants.SETPOINT_REEF_T1)));
        operatorController.povLeft().onTrue(new InstantCommand(() -> moveToSetpoint(Constants.SETPOINT_REEF_T2)));
        operatorController.povUp().onTrue(new InstantCommand(() -> moveToSetpoint(Constants.SETPOINT_REEF_T3)));
        operatorController.povRight().onTrue(new InstantCommand(() -> moveToSetpoint(Constants.SETPOINT_REEF_T4)));


    }

    public void forwardAlign() {
    }

    public CommandSwerveDrivetrain getDrivetrain() {
        return drivetrain;
    }

    public PathfinderSubsystem getPathfinderSubsystem() {
        return pathfinderSubsystem;
    }

    public NavInterfaceSubsystem getNavInterfaceSubsystem() {
        return navInterfaceSubsystem;
    }

    public ManipulatorSubsystem getManipulatorSubsystem() {
        return manipulatorSubsystem;
    }

    public ArmSubsystem getArmSubsystem() {
        return armSubsystem;
    }

    public ElevatorSubsystem getElevatorSubsystem() {
        return elevatorSubsystem;
    }

    // public Command getAutonomousCommand() {
    // return autoChooser.getSelected();
    // }

    public void doPathfind(Pose2d target) {
        PathConstraints constraints = new PathConstraints(
                3, 4, // 3 - 4
                Units.degreesToRadians(540),
                Units.degreesToRadians(720));

        System.out.println(target);

        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        pathfindingCommand = AutoBuilder.pathfindToPose(
                target,
                constraints,
                0);

        pathfindingCommand.schedule();

        System.out.println("PATHFIND TO " + target.toString() + " STARTED");
    }

    public void doPathfindToPath(String path) {
        try {

            PathPlannerPath pathData = PathPlannerPath.fromPathFile(path);

            PathConstraints constraints = new PathConstraints(
                    3, 4, // 3 - 4
                    Units.degreesToRadians(540),
                    Units.degreesToRadians(720));

            // Since AutoBuilder is configured, we can use it to build pathfinding commands
            pathfindingCommand = AutoBuilder.pathfindThenFollowPath(
                    pathData,
                    constraints);

            pathfindingCommand.schedule();

            System.out.println("PATHFIND TO " + path + " STARTED");

        } catch (Exception e) {
            DriverStation.reportError("Pathing failed: " + e.getMessage(), e.getStackTrace());
        }
    }

    public Command getPath(String id) {
        try {
            // Load the path you want to follow using its name in the GUI
            PathPlannerPath path = PathPlannerPath.fromPathFile(id);

            // Create a path following command using AutoBuilder. This will also trigger
            // event markers.
            return AutoBuilder.followPath(path);

        } catch (Exception e) {
            DriverStation.reportError("Pathing failed: " + e.getMessage(), e.getStackTrace());
            return Commands.none();
        }
    }

    public double getAxisMovementScale() {
        return (1 - (driveController.getRightTriggerAxis() * 0.9));
    }

    public void corePeriodic() {
        // If either of our analog sticks are moved, we want to disable the auto
        if (driveController.getLeftX() != 0 || driveController.getLeftY() != 0) {
            pathfinderSubsystem.stopAll();
        }

        manipulatorSubsystem.spinAt(operatorController.getLeftY());
    }
}
