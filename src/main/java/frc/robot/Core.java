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
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.digital.NavInterfaceSubsystem;
import frc.robot.subsystems.digital.PathfindCommand;
import frc.robot.subsystems.digital.PathfindCommand.Alignment;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.TunerConstants;

public class Core {
    public double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * Constants.MAX_SPEED; // kSpeedAt12Volts
                                                                                                        // desired top
                                                                                                        // speed
    public double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond) * Constants.MAX_ANGULAR_RATE; // 3/4
                                                                                                                   // of
                                                                                                                   // a
                                                                                                                   // rotation                                                                                            // per
                                                                                                                   // second
                                                                                                                   // max
                                                                                                                   // angular
                                                                                                                   // velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.02) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    // private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    // private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    // private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driveController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);

    // private final Joystick navController = new Joystick(2);
    private final Joystick navController = new Joystick(2);
    
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
    public final ManipulatorSubsystem manipulatorSubsystem = new ManipulatorSubsystem();
    public final ArmSubsystem armSubsystem = new ArmSubsystem();

    // public final NavInterfaceSubsystem navInterfaceSubsystem = new NavInterfaceSubsystem(drivetrain);

    // private final SendableChooser<Command> autoChooser;

    private Command pathfindingCommand;

    public Core() {
        registerAutoCommands();
        // autoChooser = AutoBuilder.buildAutoChooser();
        configureBindings();
        configureShuffleBoard();

        // drivetrain.setRobotPose(new Pose2d(7.5, 1.5, new Rotation2d(180 * (Math.PI / 180))));
    }

    public void registerAutoCommands() {
        // NamedCommands.registerCommand("OuttakeCommand", new Outtake(outtakeSubsystem));
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
                drivetrain.applyRequest(() -> drive.withVelocityX(-driveController.getLeftY() * MaxSpeed) // Drive
                                                                                                          // forward
                                                                                                          // with
                                                                                                          // negative Y
                                                                                                          // (forward)
                        .withVelocityY(-driveController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                        .withRotationalRate(-driveController.getRightX() * MaxAngularRate) // Drive counterclockwise
                                                                                           // with negative X (left)
                ));

        // driveController.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // driveController.b().whileTrue(drivetrain.applyRequest(() ->
        // point.withModuleDirection(new Rotation2d(-driveController.getLeftY(),
        // -driveController.getLeftX()))
        // ));


        driveController.povDown().onTrue(elevatorSubsystem.runOnce(() -> elevatorSubsystem.elevatorGoTo(1)));
        driveController.povLeft().onTrue(elevatorSubsystem.runOnce(() -> elevatorSubsystem.elevatorGoTo(2)));
        driveController.povUp().onTrue(elevatorSubsystem.runOnce(() -> elevatorSubsystem.elevatorGoTo(3)));
        driveController.povRight().onTrue(elevatorSubsystem.runOnce(() -> elevatorSubsystem.elevatorGoTo(4)));
        driveController.a().onTrue(elevatorSubsystem.runOnce(() -> elevatorSubsystem.elevatorGoTo(0)));
        driveController.b().onTrue(elevatorSubsystem.runOnce(() -> elevatorSubsystem.elevatorGoTo(5)));
        
        driveController.y().onTrue(elevatorSubsystem.runOnce(() -> elevatorSubsystem.zeroSystem()));

        driveController.leftBumper().whileTrue(elevatorSubsystem.runOnce(() -> elevatorSubsystem.lower()));
        driveController.rightBumper().whileTrue(elevatorSubsystem.runOnce(() -> elevatorSubsystem.raise()));




        operatorController.rightBumper().onTrue(armSubsystem.runOnce(() -> armSubsystem.armUp()));
        operatorController.leftBumper().onTrue(armSubsystem.runOnce(() -> armSubsystem.armDown()));

        operatorController.a().onTrue(armSubsystem.runOnce(() -> armSubsystem.rotateWristIntake()));
        operatorController.b().onTrue(armSubsystem.runOnce(() -> armSubsystem.rotateWristOuttake()));


        operatorController.y().onTrue(manipulatorSubsystem.runOnce(() -> manipulatorSubsystem.intake()));
        operatorController.x().onTrue(manipulatorSubsystem.runOnce(() -> manipulatorSubsystem.outtake()));

        operatorController.povUp().onTrue(manipulatorSubsystem.runOnce(() -> manipulatorSubsystem.stop()));


        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // driveController.back().and(driveController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // driveController.back().and(driveController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // driveController.start().and(driveController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // driveController.start().and(driveController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
        // driveController.a().onTrue(outtakeSubsystem.runOnce(() -> outtakeSubsystem.getDistance()));
        
        // reset the field-centric heading on left bumper press
        driveController.back().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        
        // drivetrain.registerTelemetry(logger::telemeterize);

        //Reef controller inputs for Teleop alignments + elevator positions
        new JoystickButton(navController, 1).onTrue(new InstantCommand(() -> {new PathfindCommand(drivetrain, 17, Alignment.LEFT);}));
        new JoystickButton(navController, 2).onTrue(new InstantCommand(() -> {new PathfindCommand(drivetrain, 17, Alignment.RIGHT);}));
        new JoystickButton(navController, 3).onTrue(new InstantCommand(() -> {new PathfindCommand(drivetrain, 22, Alignment.LEFT);}));
        new JoystickButton(navController, 4).onTrue(new InstantCommand(() -> {new PathfindCommand(drivetrain, 22, Alignment.RIGHT);}));
        new JoystickButton(navController, 5).onTrue(new InstantCommand(() -> {new PathfindCommand(drivetrain, 21, Alignment.LEFT);}));
        new JoystickButton(navController, 6).onTrue(new InstantCommand(() -> {new PathfindCommand(drivetrain, 21, Alignment.RIGHT);}));
        new JoystickButton(navController, 7).onTrue(new InstantCommand(() -> {new PathfindCommand(drivetrain, 20, Alignment.LEFT);}));
        new JoystickButton(navController, 8).onTrue(new InstantCommand(() -> {new PathfindCommand(drivetrain, 20, Alignment.RIGHT);}));
        new JoystickButton(navController, 9).onTrue(new InstantCommand(() -> {new PathfindCommand(drivetrain, 19, Alignment.LEFT);}));
        new JoystickButton(navController, 10).onTrue(new InstantCommand(() -> {new PathfindCommand(drivetrain, 19, Alignment.RIGHT);}));
        new JoystickButton(navController, 11).onTrue(new InstantCommand(() -> {new PathfindCommand(drivetrain, 18, Alignment.LEFT);}));
        new JoystickButton(navController, 12).onTrue(new InstantCommand(() -> {new PathfindCommand(drivetrain, 18, Alignment.RIGHT);}));
        // new JoystickButton(navController, 13).onTrue(elevatorSubsystem.runOnce(() -> elevatorSubsystem.elevatorGoTo(4)));
        // new JoystickButton(navController, 14).onTrue(elevatorSubsystem.runOnce(() -> elevatorSubsystem.elevatorGoTo(3)));
        // new JoystickButton(navController, 15).onTrue(elevatorSubsystem.runOnce(() -> elevatorSubsystem.elevatorGoTo(2)));
        // new JoystickButton(navController, 16).onTrue(elevatorSubsystem.runOnce(() -> elevatorSubsystem.elevatorGoTo(1)));
    }

    public void forwardAlign() {
    }

    // public Command getAutonomousCommand() {
    //     return autoChooser.getSelected();
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
}
