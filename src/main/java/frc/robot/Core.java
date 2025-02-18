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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
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
import frc.robot.commands.auto.ExactAlign;
import frc.robot.commands.auto.StaticBackCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.digital.NavInterfaceSubsystem;
import frc.robot.subsystems.digital.PathfindCommand;
import frc.robot.subsystems.digital.PathfindCommand.Alignment;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.TunerConstants;
import frc.robot.utils.Setpoint;

public class Core {

    public double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * Constants.MAX_SPEED; // kSpeedAt12Volts
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
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.02) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    // private final SwerveRequest.SwerveDriveBrake brake = new
    // SwerveRequest.SwerveDriveBrake();
    // private final SwerveRequest.PointWheelsAt point = new
    // SwerveRequest.PointWheelsAt();

    // private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driveController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);

    private final Joystick navControllerA = new Joystick(2);
    private final Joystick navControllerB = new Joystick(3);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
    public final ManipulatorSubsystem manipulatorSubsystem = new ManipulatorSubsystem();
    public final ArmSubsystem armSubsystem = new ArmSubsystem();

    // public final NavInterfaceSubsystem navInterfaceSubsystem = new
    // NavInterfaceSubsystem(drivetrain);

    // private final SendableChooser<Command> autoChooser;

    private Command pathfindingCommand;

    private String queuedRetractAction;

    public Core() {
        registerAutoCommands();
        // autoChooser = AutoBuilder.buildAutoChooser();
        configureBindings();
        configureShuffleBoard();

        // drivetrain.setRobotPose(new Pose2d(7.5, 1.5, new Rotation2d(180 * (Math.PI /
        // 180))));
    }

    public void moveToSetpoint(Setpoint setpoint) {
        queuedRetractAction = setpoint.getRetractAction();
        elevatorSubsystem.elevatorGoToDouble(setpoint.getElevator());
        armSubsystem.armGoTo(setpoint.getArm());
        armSubsystem.wristGoTo(setpoint.getWrist());
    }

    private void performRetract() {
        if (queuedRetractAction != null) {
            switch (queuedRetractAction) {
                case "none":
                    break;
                case "wristAndBackAndDown":
                    // primary steps
                    System.out.println("Running retract macro: backAndDown");
                    armSubsystem.armChangeBy(-18);
                    SequentialCommandGroup waitAndEle = new SequentialCommandGroup(new WaitCommand(2),
                            new InstantCommand(
                                    () -> elevatorSubsystem.changeBy(-Constants.RETRACT_ELEVATOR_DOWNSHIFT)));
                    waitAndEle.schedule();
                    SequentialCommandGroup waitAndOuttake = new SequentialCommandGroup(new WaitCommand(3.2),
                            new InstantCommand(() -> manipulatorSubsystem.spinAt(-0.2)), new WaitCommand(1),
                            new InstantCommand(() -> manipulatorSubsystem.stop()));
                    waitAndOuttake.schedule();
                    SequentialCommandGroup waitAndBack = new SequentialCommandGroup(new WaitCommand(3),
                            new StaticBackCommand(drivetrain, -0.4, -0.4));
                    waitAndBack.schedule();
                    break;
                case "backAndDown":
                    // primary steps
                    System.out.println("Running retract macro: backAndDown");
                    elevatorSubsystem.changeBy(-Constants.RETRACT_ELEVATOR_DOWNSHIFT);
                    SequentialCommandGroup waitAndOuttake3 = new SequentialCommandGroup(new WaitCommand(0.3),
                            new InstantCommand(() -> manipulatorSubsystem.spinAt(-0.2)), new WaitCommand(0.7),
                            new InstantCommand(() -> manipulatorSubsystem.stop()));
                    waitAndOuttake3.schedule();
                    SequentialCommandGroup waitAndBack3 = new SequentialCommandGroup(new WaitCommand(0.6),
                            new StaticBackCommand(drivetrain, -0.4, -1));
                    waitAndBack3.schedule();
                    break;
                case "tiltAndDown":
                    // primary steps
                    System.out.println("Running retract macro: backAndDown");
                    elevatorSubsystem.changeBy(-10);
                    armSubsystem.armChangeBy(-17);
                    SequentialCommandGroup waitAndOuttake2 = new SequentialCommandGroup(new WaitCommand(0.2),
                            new InstantCommand(() -> manipulatorSubsystem.spinAt(-0.32)), new WaitCommand(3),
                            new InstantCommand(() -> manipulatorSubsystem.stop()));
                    waitAndOuttake2.schedule();
                    SequentialCommandGroup waitAndBack2 = new SequentialCommandGroup(new WaitCommand(1.5),
                            new StaticBackCommand(drivetrain, -0.4, -1));
                    waitAndBack2.schedule();
                    break;
                default:
                    break;
            }
        }
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
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() -> drive
                        .withVelocityX(-driveController.getLeftY() * MaxSpeed * getAxisMovementScale()) // Drive
                        // forward
                        // with
                        // negative Y
                        // (forward)
                        .withVelocityY(-driveController.getLeftX() * MaxSpeed * getAxisMovementScale()) // Drive left
                                                                                                        // with negative
                                                                                                        // X (left)
                        .withRotationalRate(-driveController.getRightX() * MaxAngularRate * getAxisMovementScale()) // Drive
                                                                                                                    // counterclockwise
                // with negative X (left)
                ));

        // driveController.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // driveController.b().whileTrue(drivetrain.applyRequest(() ->
        // point.withModuleDirection(new Rotation2d(-driveController.getLeftY(),
        // -driveController.getLeftX()))
        // ));

        driveController.povDown().onTrue(new InstantCommand(() -> moveToSetpoint(Constants.SETPOINT_REEF_T1)));
        driveController.povLeft().onTrue(new InstantCommand(() -> moveToSetpoint(Constants.SETPOINT_REEF_T2)));
        driveController.povUp().onTrue(new InstantCommand(() -> moveToSetpoint(Constants.SETPOINT_REEF_T3)));
        driveController.povRight().onTrue(new InstantCommand(() -> moveToSetpoint(Constants.SETPOINT_REEF_T4)));

        driveController.a().onTrue(new InstantCommand(() -> moveToSetpoint(Constants.SETPOINT_MIN)));
        // driveController.y().onTrue(new InstantCommand(() ->
        // moveToSetpoint(Constants.SETPOINT_MAX)));

        driveController.b().onTrue(new InstantCommand(() -> moveToSetpoint(Constants.SETPOINT_HP_INTAKE)));

        driveController.x().onTrue(new InstantCommand(() -> performRetract()));

        // driveController.y().onTrue(elevatorSubsystem.runOnce(() -> elevatorSubsystem.zeroSystem()));

        driveController.y().onTrue(new InstantCommand(() -> new PathfindCommand(drivetrain, 19, Alignment.LEFT)));

        driveController.leftBumper().whileTrue(elevatorSubsystem.runOnce(() -> elevatorSubsystem.lower()));
        driveController.rightBumper().whileTrue(elevatorSubsystem.runOnce(() -> elevatorSubsystem.raise()));

        operatorController.rightBumper().onTrue(armSubsystem.runOnce(() -> armSubsystem.armUp()));
        operatorController.leftBumper().onTrue(armSubsystem.runOnce(() -> armSubsystem.armDown()));

        operatorController.y().onTrue(armSubsystem.runOnce(() -> armSubsystem.rotateWristIntake()));
        operatorController.x().onTrue(armSubsystem.runOnce(() -> armSubsystem.rotateWristOuttake()));

        operatorController.a().onTrue(armSubsystem.runOnce(() -> armSubsystem.wristCCW()));
        operatorController.b().onTrue(armSubsystem.runOnce(() -> armSubsystem.wristCW()));

        operatorController.povUp().onTrue(manipulatorSubsystem.runOnce(() -> manipulatorSubsystem.intake()));
        operatorController.povDown().onTrue(manipulatorSubsystem.runOnce(() -> manipulatorSubsystem.outtake()));
        operatorController.povLeft().onTrue(manipulatorSubsystem.runOnce(() -> manipulatorSubsystem.stop()));
        operatorController.povRight().onTrue(manipulatorSubsystem.runOnce(() -> manipulatorSubsystem.eject()));

        operatorController.start().onTrue(manipulatorSubsystem.runOnce(() -> manipulatorSubsystem.holdAlgae()));

        // reset the field-centric heading on left bumper press
        driveController.back().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

         driveController.a().onTrue(new InstantCommand(() -> {new PathfindCommand(drivetrain, 17, Alignment.LEFT);}));
        
        // drivetrain.registerTelemetry(logger::telemeterize);

        //Reef controller inputs for Teleop alignments + elevator positions
        new JoystickButton(navControllerA, 1).onTrue(new InstantCommand(() -> {new PathfindCommand(drivetrain, 18, Alignment.LEFT);}));
        new JoystickButton(navControllerA, 2).onTrue(new InstantCommand(() -> {new PathfindCommand(drivetrain, 18, Alignment.RIGHT);}));
        new JoystickButton(navControllerA, 3).onTrue(new InstantCommand(() -> {new PathfindCommand(drivetrain, 17, Alignment.LEFT);}));
        new JoystickButton(navControllerA, 4).onTrue(new InstantCommand(() -> {new PathfindCommand(drivetrain, 17, Alignment.RIGHT);}));
        new JoystickButton(navControllerA, 5).onTrue(new InstantCommand(() -> {new PathfindCommand(drivetrain, 22, Alignment.LEFT);}));
        new JoystickButton(navControllerA, 6).onTrue(new InstantCommand(() -> {new PathfindCommand(drivetrain, 22, Alignment.RIGHT);}));
        new JoystickButton(navControllerA, 7).onTrue(new InstantCommand(() -> {new PathfindCommand(drivetrain, 21, Alignment.LEFT);}));
        new JoystickButton(navControllerA, 8).onTrue(new InstantCommand(() -> {new PathfindCommand(drivetrain, 21, Alignment.RIGHT);}));
        new JoystickButton(navControllerA, 9).onTrue(new InstantCommand(() -> {new PathfindCommand(drivetrain, 20, Alignment.LEFT);}));
        new JoystickButton(navControllerA, 10).onTrue(new InstantCommand(() -> {new PathfindCommand(drivetrain, 20, Alignment.RIGHT);}));
        new JoystickButton(navControllerA, 11).onTrue(new InstantCommand(() -> {new PathfindCommand(drivetrain, 19, Alignment.LEFT);}));
        new JoystickButton(navControllerA, 12).onTrue(new InstantCommand(() -> {new PathfindCommand(drivetrain, 19, Alignment.RIGHT);}));
        // new JoystickButton(navController, 13).onTrue(elevatorSubsystem.runOnce(() -> elevatorSubsystem.elevatorGoTo(4)));
        // new JoystickButton(navController, 14).onTrue(elevatorSubsystem.runOnce(() -> elevatorSubsystem.elevatorGoTo(3)));
        // new JoystickButton(navController, 15).onTrue(elevatorSubsystem.runOnce(() -> elevatorSubsystem.elevatorGoTo(2)));
        // new JoystickButton(navController, 16).onTrue(elevatorSubsystem.runOnce(() -> elevatorSubsystem.elevatorGoTo(1)));

        
    }

    public void forwardAlign() {
    }

    public void instantFind(int tagId, Alignment alignment) {
        new InstantCommand(() -> {new PathfindCommand(drivetrain, 18, Alignment.LEFT);});
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
}
