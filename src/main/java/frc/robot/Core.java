// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.seafinder2.PathfinderSubsystem;
import frc.robot.seafinder2.interfaces.PanelSubsystem;
import frc.robot.seafinder2.utils.Target;
import frc.robot.seafinder2.utils.Target.TagRelativePose;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.TunerConstants;
import frc.robot.seafinder2.SF2Constants;
import frc.robot.seafinder2.commands.ExactAlign;
import frc.robot.seafinder2.commands.TestCommand;

public class Core {

    public double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * Constants.MAX_SPEED;

    public double MaxSpeedTurbo = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * Constants.MAX_SPEED_TURBO;

    public boolean isTurbo = false;

    public double currentElevatorPosition = 0;

    public double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond) * Constants.MAX_ANGULAR_RATE;

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.02) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
   
    private final CommandXboxController driveController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
    public final ManipulatorSubsystem manipulatorSubsystem = new ManipulatorSubsystem();
    // public final ClimberSubsystem climberSubsystem = new ClimberSubsystem();

    public final PathfinderSubsystem pathfinderSubsystem = new PathfinderSubsystem(this);

    public final PanelSubsystem panelSubsystem = new PanelSubsystem(pathfinderSubsystem);

    public SequentialCommandGroup autoCommandGroup;

    Pose3d llp;
    Pose3d llp2;
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

        // DEBUG

        ShuffleboardTab tab = Shuffleboard.getTab("Test");

        if (llp != null) {

            // LL Outs
            tab.addDouble("EE LL X", () -> {
                return llp.getX();
            });
            tab.addDouble("EE LL Y", () -> {
                return llp.getY();
            });
            tab.addDouble("EE LL Yaw", () -> {
                return llp.getRotation().getZ();
            });

        }
        autoCommandGroup = pathfinderSubsystem.autoSequence;
    }

    // A setpoint is a "macro" state. Find its definition in utils folder.
    // public void moveToSetpoint(Setpoint setpoint) {
    //     queuedRetractAction = setpoint.getRetractAction(); // Store what we just did for when we retract
    //     elevatorSubsystem.elevatorGoToDouble(setpoint.getElevator());
    //     armSubsystem.armGoTo(setpoint.getArm());
    //     armSubsystem.wristGoTo(setpoint.getWrist());
    // }

    public void moveToSetpoint(Target.Setpoint setpoint) {
        CommandScheduler.getInstance().schedule(elevatorSubsystem.GoTo(setpoint.getElevator()));
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

        // tab.addBoolean("IN RANGE", () -> drivetrain.robotNearHP());

        tab.addBoolean("FAST MODE", () -> {
            return isTurbo;
        });

        tab.addDouble("LEFTCR", () -> drivetrain.getForwardRangeLeft());
        tab.addDouble("RIGHTCR", () -> drivetrain.getForwardRangeRight());


        // tab.add("Auto Chooser", autoChooser);

    }

    private void configureBindings() {

        // STICK MOVEMENT
        drivetrain.setDefaultCommand(
                drivetrain.applyRequest(() -> drive
                        .withVelocityX(-driveController.getLeftY() * (isTurbo ? MaxSpeedTurbo : MaxSpeed)
                                * getAxisMovementScale() //* elevatorSlowSpeed()
                                )
                        .withVelocityY(-driveController.getLeftX() * (isTurbo ? MaxSpeedTurbo : MaxSpeed)
                                * getAxisMovementScale() //* elevatorSlowSpeed()
                                )
                        .withRotationalRate(-driveController.getRightX() * MaxAngularRate * getAxisMovementScale())));

        driveController.back().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric())); // RESET POSE
        
        driveController.a().onTrue(drivetrain.runOnce(() -> moveToSetpoint(SF2Constants.SETPOINT_ALGAE_T2))); // RESET POSE
        driveController.b().onTrue(drivetrain.runOnce(() -> moveToSetpoint(SF2Constants.SETPOINT_ALGAE_T3))); // RESET POSE
        
        driveController.povLeft().onTrue(new InstantCommand(() -> {isTurbo = !isTurbo;}));
        
        TagRelativePose testingTagRelativePose = new TagRelativePose(17, 0.52
        , 0.15, 0.0); // idk what units this is in, negative x is right
        // right & y is front back
       
        driveController.x().onTrue(new TestCommand(drivetrain));

        driveController.leftBumper().whileTrue(elevatorSubsystem.incrementUp(1));
        driveController.rightBumper().whileTrue(elevatorSubsystem.incrementDown(1));

       
        operatorController.a().onTrue(new InstantCommand(() -> moveToSetpoint(SF2Constants.SETPOINT_MIN)));
        ParallelCommandGroup pq = new ParallelCommandGroup(elevatorSubsystem.GoTo(112));

        ParallelCommandGroup pq2 = new ParallelCommandGroup(elevatorSubsystem.GoTo(65));

        SequentialCommandGroup sq = new SequentialCommandGroup(pq, new ExactAlign(drivetrain, testingTagRelativePose), pq2);

        driveController.y().onTrue(sq);

        operatorController.povDown().onTrue(new InstantCommand(() -> moveToSetpoint(SF2Constants.SETPOINT_REEF_T1)));
        operatorController.povLeft().onTrue(new InstantCommand(() -> moveToSetpoint(SF2Constants.SETPOINT_REEF_T2)));
        operatorController.povUp().onTrue(new InstantCommand(() -> moveToSetpoint(SF2Constants.SETPOINT_REEF_T3)));
        operatorController.povRight().onTrue(new InstantCommand(() -> moveToSetpoint(SF2Constants.SETPOINT_REEF_T4)));
    }

    public void forwardAlign() {
    }

    public CommandSwerveDrivetrain getDrivetrain() {
        return drivetrain;
    }

    public PathfinderSubsystem getPathfinderSubsystem() {
        return pathfinderSubsystem;
    }

    public ManipulatorSubsystem getManipulatorSubsystem() {
        return manipulatorSubsystem;
    }

    public ElevatorSubsystem getElevatorSubsystem() {
        return elevatorSubsystem;
    }

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
        return (1 - (driveController.getRightTriggerAxis() * 0.85));
    }

    public double elevatorSlowSpeed() {
        if (currentElevatorPosition > 20) {
            return 20 / currentElevatorPosition;
        }

        return 1;
    }

    int clock = 0;

    public void corePeriodic() {
        clock++;
        if (clock > 10) {

            clock = 0;
        }
        // If either of our analog sticks are moved, we want to disable the auto
        if (driveController.getLeftX() != 0 || driveController.getLeftY() != 0) {
            pathfinderSubsystem.stopAll();
            if (autoCommandGroup != null) {
                autoCommandGroup.cancel();
            }
        }

        if (manipulatorSubsystem.overriding == true && operatorController.getLeftY() == 0) {
            return;
        }

        manipulatorSubsystem.overriding = false;
        if (pathfinderSubsystem.intakeCommand != null) {
            pathfinderSubsystem.intakeCommand.cancel();
        }

        if ((operatorController.getLeftY() < 0)) {
            if (operatorController.x().getAsBoolean()) {
                manipulatorSubsystem.spinAt(operatorController.getLeftY());
            } else {
                manipulatorSubsystem.spinAt(operatorController.getLeftY() / 4);
            }
        } else {
            manipulatorSubsystem.spinAt(operatorController.getLeftY());
        }

        currentElevatorPosition = elevatorSubsystem.mechanismHeight();
    }
}
